// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include "battery_discharge.hh"

#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
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

    this->sim_time_now = this->world->SimTime();

    // Create ros node and publish stuff there!
    this->rosNode = std::make_unique<ros::NodeHandle>(_sdf->Get<std::string>("ros_node"));

    // Publish a topic for charge level
    this->charge_state = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/charge_level", 1);
    this->charge_state_mwh = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/charge_level_mwh", 1);

    this->set_charging = this->rosNode->advertiseService(
        this->model->GetName() + "/set_charging", &BatteryPlugin::SetCharging, this);
    this->set_charging_rate = this->rosNode->advertiseService(
        this->model->GetName() + "/set_charge_rate", &BatteryPlugin::SetChargingRate, this);
    this->set_charge = this->rosNode->advertiseService(
        this->model->GetName() + "/set_charge", &BatteryPlugin::SetCharge, this);
    this->set_coefficients = this->rosNode->advertiseService(
        this->model->GetName() + "/set_model_coefficients", &BatteryPlugin::SetModelCoefficients, this);

    const auto linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->qt = _sdf->Get<double>("charge_rate");
    this->c = _sdf->Get<double>("capacity");
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");

    const auto batteryName = _sdf->Get<std::string>("battery_name");

    if (this->link->BatteryCount() > 0)
    {
        // Creates the battery
        this->battery = this->link->Battery(batteryName);
        gzlog << "Created battery" << batteryName << ".\n";
    } else
    {
        gzerr << "There is no battery specification in the link!\n";
        return;
    };

    // Specifying a custom update function
    this->battery->SetUpdateFunc([this](const common::BatteryPtr& b)
                                 { return this->OnUpdateVoltage(b); });

    this->sim_time_now = this->world->SimTime();

    gzlog << "BatteryPlugin Loaded.\n";
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

    if (fabs(_battery->Voltage()) < 1e-3)
        return 0.0;

    for (auto powerLoad : _battery->PowerLoads())
        totalpower += powerLoad.second;

    // current = power(Watts)/Voltage
    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

    std_msgs::Float64 charge_msg, charge_msg_mwh;
    {
        std::lock_guard<std::mutex> l(this->lock);

        if (!this->charging)
        {
            this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
        } else
        {
            this->q = this->q + GZ_SEC_TO_HOUR(dt * this->qt);
        }

        this->sim_time_now = this->world->SimTime();

#ifdef BATTERY_DEBUG
        gzdbg << "Current charge:" << this->q << ", at:" << this->sim_time_now << "\n";
#endif

        this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

#ifdef BATTERY_DEBUG
        gzdbg << "Current voltage:" << this->et << ", at:" << this->sim_time_now << "\n";
#endif

        // Turn off the motor
        if (this->q <= 0)
        {
            this->sim_time_now = this->world->SimTime();

            // TODO figure out how to turn off the robot

#ifdef BATTERY_DEBUG
            gzdbg << "Out of juice at:" << this->sim_time_now << "\n";
#endif

        } else if (this->q >= this->c)
        {
            this->q = this->c;
        }

        charge_msg.data = this->q;
        charge_msg_mwh.data = this->q * 1000 * this->et;
    }

    this->charge_state.publish(charge_msg);
    this->charge_state_mwh.publish(charge_msg_mwh);

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
    } else
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
    } else
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
