#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <limits>
#include <memory>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <gazebo_ros_battery/SetLoad.h>

namespace gazebo
{

class GAZEBO_VISIBLE CmdVelConsumerPlugin : public ModelPlugin
{
public:
    CmdVelConsumerPlugin();

    ~CmdVelConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr& _msg);

private:
    double CalculatePower(const geometry_msgs::Twist::ConstPtr& _msg);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;

    double powerLoadRate {0.0};  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.

private:
    common::BatteryPtr battery;

    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier.

protected:
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_power_pub;
};
}
