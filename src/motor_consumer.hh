#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <thread>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/JointState.h"

#include "gazebo_ros_battery/SetLoad.h"

namespace gazebo
{

class GAZEBO_VISIBLE MotorConsumerPlugin : public ModelPlugin
{
public:
    MotorConsumerPlugin();

    ~MotorConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Init() override;

    void Reset() override;

    void OnJointStateMsg(const sensor_msgs::JointState::ConstPtr& _msg);

private:
    void QueueThread();

    double CalculatePower(const sensor_msgs::JointState::ConstPtr& _msg);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;

    // Consumer parameter
    double powerLoadRate;
    double consumerIdlePower;

private:
    // Battery
    common::BatteryPtr battery;

    // Consumer identifier
    int32_t consumerId;

protected:
    double powerLoad;

    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber joint_state_sub;
    ros::Publisher motor_power_pub;

    boost::mutex lock;

private:
    ros::CallbackQueue rosQueue;

    std::thread rosQueueThread;
};
}
