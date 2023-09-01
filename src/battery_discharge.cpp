// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - changed to publish BatteryState message
// - removed the services
// - added temperature modeling

#include "battery_discharge.hh"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float64.h>

// #define BATTERY_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin() = default;

BatteryPlugin::~BatteryPlugin()
{
    if (this->rosNode)
        this->rosNode->shutdown();
}

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the "
              << "Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.\n";
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string robotNamespace;
    if (_sdf->HasElement("robotNamespace"))
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);

    this->gzNode.reset(new gazebo::transport::Node);
    this->gzNode->Init(_model->GetWorld()->Name());

    this->updatePeriod = 1.0 / _sdf->Get<double>("update_rate", 1 / this->updatePeriod).first;

    const auto gzNs = "~/" + _model->GetName() + "/" + _sdf->GetAttribute("name")->GetAsString() + "/";
    this->gzChargePowerPub = this->gzNode->Advertise<msgs::Any>(gzNs + "charge_power", 1, 1 / this->updatePeriod);
    this->gzDischargePowerPub = this->gzNode->Advertise<msgs::Any>(gzNs + "discharge_power", 1, 1 / this->updatePeriod);

    this->battery_state = this->rosNode->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    this->charge_state_wh = this->rosNode->advertise<std_msgs::Float64>("charge_level_wh", 1);

    this->ambientTemperature = _sdf->Get<double>("ambient_temperature", this->ambientTemperature).first;
    const auto ambientTemperatureRosTopic = _sdf->Get<std::string>("ambient_temperature_ros_topic", "").first;
    const auto ambientTemperatureGzTopic = _sdf->Get<std::string>("ambient_temperature_gz_topic", "").first;
    if (!ambientTemperatureRosTopic.empty())
        this->rosAmbientTemperatureSub = this->rosNode->subscribe(
            ambientTemperatureRosTopic, 10, &BatteryPlugin::OnRosAmbientTempMsg, this);
    else if (!ambientTemperatureGzTopic.empty())
        this->gzAmbientTemperatureSub = this->gzNode->Subscribe(
            ambientTemperatureGzTopic, &BatteryPlugin::OnGzAmbientTempMsg, this, true);

    const auto linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->allowCharging = _sdf->Get<bool>("allow_charging", this->allowCharging).first;
    const auto allowChargingGzTopic = _sdf->Get<std::string>(
        "allow_charging_gz_topic", "~/" + _model->GetName() + "/allow_charging").first;
    if (!allowChargingGzTopic.empty())
        this->gzAllowChargingSub = this->gzNode->Subscribe(
            allowChargingGzTopic, &BatteryPlugin::OnGzAllowChargingMsg, this, true);

    this->computeResistance = _sdf->Get<bool>("compute_resistance", false).first;
    this->computeTemperature = _sdf->Get<bool>("compute_temperature", false).first;

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->c = _sdf->Get<double>("capacity");
    const auto designCapacity = _sdf->Get<double>("design_capacity", this->c).first;
    this->tau = _sdf->Get<double>("smooth_current_tau");

    this->t = this->baseTemperature = _sdf->Get<double>("temperature", this->t).first;

    if (this->computeResistance)
    {
        const auto coefs = _sdf->Get<std::string>("resistance_temperature_coeffs", "").first;
        std::vector<std::string> coeffStrs;
        boost::split(coeffStrs, coefs, boost::is_any_of(",;"));
        for (const auto& coeffStr : coeffStrs)
        {
            try
            {
                this->resistanceTemperatureCoeffs.emplace_back(boost::lexical_cast<double>(coeffStr));
            }
            catch (const boost::bad_lexical_cast& e)
            {
                gzerr << "Could not read coefficient " << coeffStr << " as number.\n";
            }
        }
    }
    else
    {
        this->r = _sdf->Get<double>("resistance");
    }

    this->heatDissipationRate = _sdf->Get<double>("heat_dissipation_rate", this->heatDissipationRate).first;
    this->heatCapacity = _sdf->Get<double>("heat_capacity", this->heatCapacity).first;

    const auto batteryName = _sdf->Get<std::string>("battery_name");

    if (this->link->BatteryCount() > 0)
    {
        this->battery = this->link->Battery(batteryName);
    }
    else
    {
        gzerr << "There is no battery specification in the link!\n";
        return;
    }

    const auto frameId = _sdf->Get<std::string>("frame_id", batteryName).first;

    const auto numCells = _sdf->Get<size_t>("num_cells", 0u).first;
    this->reportCellVoltage = _sdf->Get<bool>("report_cell_voltage", this->reportCellVoltage).first;
    this->reportCellTemperature = _sdf->Get<bool>("report_cell_temperature", false).first;
    const auto batteryLocation = _sdf->Get<std::string>("location", "").first;
    const auto batterySerial = _sdf->Get<std::string>("serial_number", "").first;
    const auto technology = _sdf->Get<std::string>("technology", "").first;
    auto powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    if (technology == "NIMH")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
    else if (technology == "LION")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    else if (technology == "LIPO")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    else if (technology == "LIFE")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
    else if (technology == "NICD")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD;
    else if (technology == "LIMN")
        powerSupplyTechnology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN;
    else if (!technology.empty())
        gzerr << "Unknown battery technology " << technology << "!\n";

    this->batteryMsg.header.frame_id = frameId;
    this->batteryMsg.capacity = static_cast<float>(this->c);
    this->batteryMsg.design_capacity = static_cast<float>(designCapacity);
    this->batteryMsg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    this->batteryMsg.power_supply_technology = powerSupplyTechnology;
    this->batteryMsg.present = true;
    this->batteryMsg.location = batteryLocation;
    this->batteryMsg.serial_number = batterySerial;
#if ROS_VERSION_MINIMUM(1, 15, 0)
    this->batteryMsg.temperature = this->t;
#endif
    this->batteryMsg.cell_voltage.resize(numCells, std::numeric_limits<float>::quiet_NaN());
#if ROS_VERSION_MINIMUM(1, 15, 0)
    this->batteryMsg.cell_temperature.resize(numCells,
        this->reportCellTemperature ? this->t : std::numeric_limits<float>::quiet_NaN());
#endif

    // Specifying a custom update function
    this->battery->SetUpdateFunc([this](const common::BatteryPtr& b)
                                 { return this->OnUpdateVoltage(b); });

    gzmsg << "Loaded battery '" << linkName << "/" << batteryName << "' (" << this->battery->Voltage() << " V, "
          << this->c << " Ah).\n";
}

void BatteryPlugin::Init()
{
    this->q = this->q0;
    this->heatEnergy = 0.0;
    if (this->computeTemperature)
        this->t = this->baseTemperature;
    this->UpdateResistance();
}

void BatteryPlugin::Reset()
{
    this->ismooth = 0.0;
    this->lastUpdateTime = common::Time::Zero;
    this->Init();
    gzdbg << "Battery '" << this->battery->Name() << "' was reset.\n";
}

void BatteryPlugin::UpdateResistance()
{
    if (this->computeResistance && !std::isnan(this->t) && !this->resistanceTemperatureCoeffs.empty())
    {
        const auto& coefs = this->resistanceTemperatureCoeffs;
        double newR = 0;
        for (size_t i = 0; i < coefs.size(); ++i)
            newR += coefs[i] * std::pow(this->t, i);
        this->r = newR;

#ifdef BATTERY_DEBUG
        gzdbg << "Current resistance:" << this->r << ", at:" << this->world->SimTime() << "\n";
#endif
    }
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr& _battery)
{
    double dt = this->world->Physics()->GetMaxStepSize();

    double totalPower {0.0};
    double totalChargePower {0.0};
    double totalDischargePower {0.0};
    double k = dt / this->tau;

    for (auto powerLoad : _battery->PowerLoads())
    {
        if (powerLoad.second >= 0 || this->allowCharging)
            totalPower += powerLoad.second;
        if (powerLoad.second >= 0)
            totalDischargePower += powerLoad.second;
        if (this->allowCharging && powerLoad.second < 0)
            totalChargePower += -powerLoad.second;
    }

    // Do not let the voltage drop under the minimum value.
    const auto voltage = (std::max)(_battery->Voltage(), this->e0 + this->e1);

    // current = power(Watts)/Voltage
    const auto iraw = totalPower / voltage;  // Raw battery current in A.

    this->ismooth = this->ismooth + k * (iraw - this->ismooth);

    this->q = (std::max)(0.0, this->q - GZ_SEC_TO_HOUR(dt * this->ismooth));

#ifdef BATTERY_DEBUG
    gzdbg << "Current charge:" << this->q << ", at:" << this->world->SimTime() << "\n";
#endif

    this->UpdateResistance();

    // Voltage on terminal
    const auto voltageLoss = this->r * this->ismooth;
    double et = this->e0 + this->e1 * (1 - this->q / this->c) - voltageLoss;
    // In case the battery is charging (ismooth is negative), we don't want the voltage to go over the maximum
    et = (std::min)(et, this->e0);

#ifdef BATTERY_DEBUG
    gzdbg << "Current voltage:" << et << ", at:" << this->world->SimTime() << "\n";
#endif

    // Watts of power lost due to internal resistance
    auto powerLoss = voltageLoss * this->ismooth;  // note this is always non-negative (contains I^2)

    // Turn off the motor
    if (this->q <= 0)
    {
        et = 0;
        powerLoss = 0;
        totalDischargePower = std::min(totalDischargePower, totalChargePower);

        // TODO figure out how to turn off the robot

#ifdef BATTERY_DEBUG
        gzdbg << "Out of juice at:" << this->world->SimTime() << "\n";
#endif
    }
    else if (this->q >= this->c)
    {
        this->q = this->c;
        // If the battery is charging and full, do not let any other charging current in
        this->ismooth = (std::max)(0.0, this->ismooth);

        powerLoss = voltageLoss * this->ismooth;
        totalChargePower = 0;
    }

    if (this->computeTemperature)
    {
        const auto generatedHeatJoules = powerLoss * dt;
        this->heatEnergy += generatedHeatJoules;

        const auto dissipatedJoules = (this->t - this->ambientTemperature) * this->heatDissipationRate;
        this->heatEnergy -= dissipatedJoules;

        this->t = this->baseTemperature + this->heatEnergy / this->heatCapacity;

#ifdef BATTERY_DEBUG
        gzdbg << "Current temperature:" << this->t << ", at:" << this->world->SimTime() << "\n";
#endif
    }

    if (this->lastUpdateTime + this->updatePeriod < this->world->SimTime())
    {
        this->lastUpdateTime = this->world->SimTime();

        this->batteryMsg.header.stamp.sec = this->lastUpdateTime.sec;
        this->batteryMsg.header.stamp.nsec = this->lastUpdateTime.nsec;
        this->batteryMsg.voltage = static_cast<float>(et);
        this->batteryMsg.current = static_cast<float>(-this->ismooth);
        this->batteryMsg.charge = static_cast<float>(this->q);
        this->batteryMsg.percentage = static_cast<float>(this->q / this->c);
        if (this->q == this->c)
            this->batteryMsg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
        else if (this->ismooth >= 0)
            this->batteryMsg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        else
            this->batteryMsg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        if (this->reportCellVoltage && !this->batteryMsg.cell_voltage.empty())
        {
            const auto cellVoltage = static_cast<float>(et / static_cast<double>(this->batteryMsg.cell_voltage.size()));
            std::fill(this->batteryMsg.cell_voltage.begin(), this->batteryMsg.cell_voltage.end(), cellVoltage);
        }
#if ROS_VERSION_MINIMUM(1, 15, 0)
        this->batteryMsg.temperature = this->t;
        if (this->reportCellTemperature)
            std::fill(this->batteryMsg.cell_temperature.begin(), this->batteryMsg.cell_temperature.end(), this->t);
#endif

        this->battery_state.publish(this->batteryMsg);

        std_msgs::Float64 charge_msg_wh;
        charge_msg_wh.data = this->q * et;
        this->charge_state_wh.publish(charge_msg_wh);

        this->gzChargePowerPub->Publish(msgs::ConvertAny(totalChargePower));
        this->gzDischargePowerPub->Publish(msgs::ConvertAny(totalDischargePower));
    }

    return et;
}

void BatteryPlugin::OnRosAmbientTempMsg(const sensor_msgs::Temperature& _msg)
{
    this->ambientTemperature = _msg.temperature;
}

void BatteryPlugin::OnGzAmbientTempMsg(const ConstAnyPtr& _msg)
{
    if (_msg->has_type() && _msg->type() == msgs::Any_ValueType_DOUBLE && _msg->has_double_value())
        this->ambientTemperature = _msg->double_value();
}

void BatteryPlugin::OnGzAllowChargingMsg(const ConstAnyPtr& _msg)
{
    if (_msg->has_type() && _msg->type() == msgs::Any_ValueType_BOOLEAN && _msg->has_bool_value())
        this->allowCharging = _msg->bool_value();
}
