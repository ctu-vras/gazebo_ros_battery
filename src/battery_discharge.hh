#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <map>
#include <string>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include <boost/thread/mutex.hpp>
#include "std_msgs/Bool.h"
#include "gazebo_ros_battery/SetCharge.h"
#include "gazebo_ros_battery/SetCharging.h"
#include "gazebo_ros_battery/SetCoef.h"
#include "gazebo_ros_battery/SetChargingRate.h"

namespace gazebo
{

/// \brief A plugin that simulate BRASS CP1 battery model: discharge and charge according to power models
class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
{
    /// \brief Constructor
public:
    BatteryPlugin();

    ~BatteryPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Init() override;

    void Reset() override;

private:
    double OnUpdateVoltage(const common::BatteryPtr& _battery);

public:
    bool SetCharging(gazebo_ros_battery::SetCharging::Request& req,
                     gazebo_ros_battery::SetCharging::Response& res);

    bool SetCharge(gazebo_ros_battery::SetCharge::Request& req,
                   gazebo_ros_battery::SetCharge::Response& res);

    bool SetModelCoefficients(gazebo_ros_battery::SetCoef::Request& req,
                              gazebo_ros_battery::SetCoef::Response& res);

    bool SetChargingRate(gazebo_ros_battery::SetChargingRate::Request& req,
                         gazebo_ros_battery::SetChargingRate::Response& res);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    common::BatteryPtr battery;
    sdf::ElementPtr sdf;

    // E(t) = e0 + e1* Q(t)/c
    double et;
    double e0;
    double e1;

    // Initial battery charge in Ah.
    double q0;

    // Charge rate in A
    // More description about charge rate: http://batteriesbyfisher.com/determining-charge-time
    double qt;

    // Battery capacity in Ah.
    double c;

    // Battery inner resistance in Ohm
    double r;

    // Current low-pass filter characteristic time in seconds.
    double tau;

    // Raw battery current in A.
    double iraw;

    // Smoothed battery current in A.
    double ismooth;

    // Instantaneous battery charge in Ah.
    double q;

    // This node is for ros communications
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher charge_state;
    ros::Publisher charge_state_mwh;
    ros::Publisher motor_power;

    ros::ServiceServer set_charging;
    ros::ServiceServer set_charge;
    ros::ServiceServer set_coefficients;
    ros::ServiceServer set_charging_rate;

    boost::mutex lock;

    bool charging;

    double sim_time_now;
};
}
