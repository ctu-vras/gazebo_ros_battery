// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - changed to publish BatteryState message
// - removed the services

#include "battery_discharge.hh"

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
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

    this->battery_state = this->rosNode->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    this->charge_state_wh = this->rosNode->advertise<std_msgs::Float64>("charge_level_wh", 1);

    const auto linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->allowCharging = _sdf->Get<bool>("allow_charging", this->allowCharging).first;

    this->updatePeriod = 1.0 / _sdf->Get<double>("update_rate", this->updatePeriod).first;
    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->c = _sdf->Get<double>("capacity");
    const auto designCapacity = _sdf->Get<double>("design_capacity", this->c).first;
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");

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

    const auto temperature = _sdf->Get<double>("temperature", std::numeric_limits<double>::quiet_NaN()).first;
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
    this->batteryMsg.temperature = temperature;
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
}

void BatteryPlugin::Reset()
{
    this->ismooth = 0.0;
    this->lastUpdateTime = common::Time::Zero;
    this->Init();
    gzdbg << "Battery '" << this->battery->Name() << "' was reset.\n";
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr& _battery)
{
    double dt = this->world->Physics()->GetMaxStepSize();

    double totalpower = 0.0;
    double k = dt / this->tau;

    for (auto powerLoad : _battery->PowerLoads())
    {
        if (powerLoad.second >= 0 || this->allowCharging)
            totalpower += powerLoad.second;
    }

    // Do not let the voltage drop under the minimum value.
    const auto voltage = (std::max)(_battery->Voltage(), this->e0 + this->e1);

    // current = power(Watts)/Voltage
    const auto iraw = totalpower / voltage;  // Raw battery current in A.

    this->ismooth = this->ismooth + k * (iraw - this->ismooth);

    this->q = (std::max)(0.0, this->q - GZ_SEC_TO_HOUR(dt * this->ismooth));

#ifdef BATTERY_DEBUG
    gzdbg << "Current charge:" << this->charge << ", at:" << this->sim_time_now << "\n";
#endif

    // Voltage on terminal
    double et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;
    // In case the battery is charging (ismooth is negative), we don't want the voltage to go over the maximum
    et = (std::min)(et, this->e0);

#ifdef BATTERY_DEBUG
    gzdbg << "Current voltage:" << this->et << ", at:" << this->sim_time_now << "\n";
#endif

    // Turn off the motor
    if (this->q <= 0)
    {
        et = 0;

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

        this->battery_state.publish(this->batteryMsg);

        std_msgs::Float64 charge_msg_wh;
        charge_msg_wh.data = this->q * et;
        this->charge_state_wh.publish(charge_msg_wh);
    }

    return et;
}
