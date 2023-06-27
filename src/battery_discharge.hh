#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <memory>
#include <mutex>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <gazebo_ros_battery/SetCharge.h>
#include <gazebo_ros_battery/SetCharging.h>
#include <gazebo_ros_battery/SetCoef.h>
#include <gazebo_ros_battery/SetChargingRate.h>

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
    double et {0.0};
    double e0 {0.0};
    double e1 {0.0};

    double q0 {0.0};  //!< Initial battery charge in Ah.

    double qt {0.0};  //!< Charge rate in A

    double c {0.0};  //!< Battery capacity in Ah.

    double r {0.0};  //!< Battery inner resistance in Ohm

    double tau {0.0};  //!< Current low-pass filter characteristic time in seconds.

    double iraw {0.0};  //!< Raw battery current in A.

    double ismooth {0.0};  //!< Smoothed battery current in A.

    double q {0.0};  //!< Instantaneous battery charge in Ah.

    // This node is for ros communications
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher charge_state;
    ros::Publisher charge_state_mwh;
    ros::Publisher motor_power;

    ros::ServiceServer set_charging;
    ros::ServiceServer set_charge;
    ros::ServiceServer set_coefficients;
    ros::ServiceServer set_charging_rate;

    std::mutex lock;

    bool charging {false};

    common::Time sim_time_now;
};
}
