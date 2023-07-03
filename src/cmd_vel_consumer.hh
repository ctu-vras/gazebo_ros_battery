#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - extracted base class BatteryConsumerBase

#include <gazebo/common/common.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/msgs/twist.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <gazebo_ros_battery/battery_consumer_base.hh>

namespace gazebo
{

class GAZEBO_VISIBLE CmdVelConsumerPlugin : public BatteryConsumerBase
{
public:
    CmdVelConsumerPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    void OnCmdVelMsg(const geometry_msgs::Twist& _msg);
    void OnGzTwistMsg(const ConstTwistPtr& _msg);
    void OnGzPoseMsg(const ConstPosePtr& _msg);
    void OnUpdate(const common::UpdateInfo& _info);

protected:
    double CalculatePower(const geometry_msgs::Twist& _msg);

    msgs::Twist powerLoadRates;  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.
    double commandDuration {0.1};  //!< For how long a velocity command is valid.

    common::Time lastCmdTime;

    transport::NodePtr gzNode;

    ros::Subscriber cmd_vel_sub;
    transport::SubscriberPtr gz_twist_sub;
    transport::SubscriberPtr gz_pose_sub;
    event::ConnectionPtr beforePhysicsUpdateConnection;
};
}
