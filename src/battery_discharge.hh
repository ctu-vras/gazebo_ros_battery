#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - changed to publish BatteryState message
// - removed the services

#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

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

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    common::BatteryPtr battery;
    sdf::ElementPtr sdf;
    common::Time lastUpdateTime;
    double updatePeriod {1.0};
    bool allowCharging {true};

    // E(t) = e0 + e1* Q(t)/c
    double e0 {0.0};
    double e1 {0.0};

    double q0 {0.0};  //!< Initial battery charge in Ah.

    double c {0.0};  //!< Battery capacity in Ah.

    double r {0.0};  //!< Battery inner resistance in Ohm

    double tau {0.0};  //!< Current low-pass filter characteristic time in seconds.

    double ismooth {0.0};  //!< Smoothed battery current in A.

    double q {0.0};  //!< Instantaneous battery charge in Ah.

    // This node is for ros communications
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher battery_state;
    ros::Publisher charge_state_wh;

    sensor_msgs::BatteryState batteryMsg;
};
}
