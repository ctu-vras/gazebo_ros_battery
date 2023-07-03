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
#include <string>
#include <unordered_map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{

class GAZEBO_VISIBLE MotorConsumerPlugin : public ModelPlugin
{
public:
    MotorConsumerPlugin();

    ~MotorConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    void OnJointStateMsg(const sensor_msgs::JointState::ConstPtr& _msg);

private:
    double CalculatePower(const sensor_msgs::JointState::ConstPtr& _msg);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    std::unordered_map<std::string, physics::JointPtr> joints;
    sdf::ElementPtr sdf;

    double efficiency {1.0};  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.

private:
    common::BatteryPtr battery;
    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier

protected:
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber joint_state_sub;
    ros::Publisher motor_power_pub;
};
}
