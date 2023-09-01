#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - extracted base class BatteryConsumerBase

#include <string>
#include <unordered_map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <gazebo_ros_battery/battery_consumer_base.hh>

namespace gazebo
{

class GAZEBO_VISIBLE MotorConsumerPlugin : public BatteryConsumerBase
{
public:
    MotorConsumerPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    void OnJointStateMsg(const sensor_msgs::JointState::ConstPtr& _msg);

protected:
    double CalculatePower(const sensor_msgs::JointState::ConstPtr& _msg);
    void SetEnabled(bool enabled) override;

    event::ConnectionPtr updateConnection;
    std::unordered_map<std::string, physics::JointPtr> joints;

    double efficiency {1.0};  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.

    double lastPowerLoad {0.0};

    ros::Subscriber joint_state_sub;
    ros::Publisher motor_power_pub;
};

}
