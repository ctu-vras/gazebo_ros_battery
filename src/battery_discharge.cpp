// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - changed to publish BatteryState message

#include "battery_discharge.hh"

#include <memory>
#include <mutex>
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
        ROS_FATAL_STREAM_NAMED("gazebo_ros_battery",
                               "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the "
                               "Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    auto robotNamespace = _model->GetName();
    if (_sdf->HasElement("robotNamespace"))
    {
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
        if (robotNamespace.empty()) robotNamespace = _model->GetName();
    }
    if (!robotNamespace.empty()) robotNamespace += "/";

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);

    this->battery_state = this->rosNode->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    this->charge_state_wh = this->rosNode->advertise<std_msgs::Float64>("charge_level_wh", 1);

    this->set_charging = this->rosNode->advertiseService("set_charging", &BatteryPlugin::SetCharging, this);
    this->set_charging_rate = this->rosNode->advertiseService("set_charge_rate", &BatteryPlugin::SetChargingRate, this);
    this->set_charge = this->rosNode->advertiseService("set_charge", &BatteryPlugin::SetCharge, this);
    this->set_coefficients = this->rosNode->advertiseService(
        "set_model_coefficients", &BatteryPlugin::SetModelCoefficients, this);

    const auto linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->updatePeriod = 1.0 / _sdf->Get<double>("update_rate", 1.0).first;
    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->qt = _sdf->Get<double>("charge_rate");
    this->c = _sdf->Get<double>("capacity");
    const auto designCapacity = _sdf->Get<double>("capacity", this->c).first;
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

    // Specifying a custom update function
    this->battery->SetUpdateFunc([this](const common::BatteryPtr& b)
                                 { return this->OnUpdateVoltage(b); });

    gzmsg << "Loaded battery '" << linkName << "/" << batteryName << "' (" << this->battery->Voltage() << " V, "
          << this->c << " Ah).\n";
}

void BatteryPlugin::Init()
{
    this->q = this->q0;
    this->charging = false;
}

void BatteryPlugin::Reset()
{
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr& _battery)
{
    double dt = this->world->Physics()->GetMaxStepSize();

    double totalpower = 0.0;
    double k = dt / this->tau;

    for (auto powerLoad : _battery->PowerLoads())
        totalpower += powerLoad.second;

    // current = power(Watts)/Voltage
    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

    double et;  // Voltage on terminal
    double charge;
    bool wasCharging;
    {
        std::lock_guard<std::mutex> l(this->lock);

        wasCharging = this->charging;

        if (!this->charging)
        {
            this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
        }
        else
        {
            this->q = this->q + GZ_SEC_TO_HOUR(dt * this->qt);
        }

#ifdef BATTERY_DEBUG
        gzdbg << "Current charge:" << this->charge << ", at:" << this->sim_time_now << "\n";
#endif

        et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

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
        }

        charge = this->q;
    }

    if (this->lastUpdateTime + this->updatePeriod < this->world->SimTime())
    {
        this->lastUpdateTime = this->world->SimTime();

        this->batteryMsg.header.stamp.sec = this->lastUpdateTime.sec;
        this->batteryMsg.header.stamp.nsec = this->lastUpdateTime.nsec;
        this->batteryMsg.voltage = static_cast<float>(et);
        this->batteryMsg.current = static_cast<float>(wasCharging ? this->qt : -this->ismooth);
        this->batteryMsg.charge = static_cast<float>(charge);
        this->batteryMsg.percentage = static_cast<float>(charge / this->c);
        this->batteryMsg.power_supply_status = wasCharging ?
                                               sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
                                               sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

        this->battery_state.publish(this->batteryMsg);

        std_msgs::Float64 charge_msg_wh;
        charge_msg_wh.data = charge * et;
        this->charge_state_wh.publish(charge_msg_wh);
    }

    return et;
}

bool BatteryPlugin::SetCharging(gazebo_ros_battery::SetCharging::Request& req,
                                gazebo_ros_battery::SetCharging::Response& res)
{
    {
        std::lock_guard<std::mutex> l(this->lock);
        this->charging = req.charging;
    }
    if (req.charging)
    {
        gzdbg << "Battery is charging.\n";
    }
    else
    {
        gzdbg << "Battery stopped charging.\n";
    }
    res.result = true;
    return true;
}

bool BatteryPlugin::SetChargingRate(gazebo_ros_battery::SetChargingRate::Request& req,
                                    gazebo_ros_battery::SetChargingRate::Response& res)
{
    {
        std::lock_guard<std::mutex> l(this->lock);
        this->qt = req.charge_rate;
    }
    gzdbg << "Charging rate has been changed to: " << req.charge_rate << "\n";
    res.result = true;
    return true;
}


bool BatteryPlugin::SetCharge(gazebo_ros_battery::SetCharge::Request& req,
                              gazebo_ros_battery::SetCharge::Response& res)
{
    std::lock_guard<std::mutex> l(this->lock);
    if (req.charge <= this->c)
    {
        this->q = req.charge;
        gzdbg << "Received charge:" << this->q << "\n";
    }
    else
    {
        this->q = this->c;
        gzerr << "The charge cannot be higher than the capacity of the battery!\n";
    }
    res.result = true;
    return true;
}

bool BatteryPlugin::SetModelCoefficients(gazebo_ros_battery::SetCoef::Request& req,
                                         gazebo_ros_battery::SetCoef::Response& res)
{
    {
        std::lock_guard<std::mutex> l(this->lock);
        this->e0 = req.constant_coef;
        this->e1 = req.linear_coef;
    }
    gzdbg << "Power model is changed, new coefficients (constant, linear):" << req.constant_coef
          << req.linear_coef << "\n";
    res.result = true;
    return true;
}
