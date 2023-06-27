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
#include "geometry_msgs/Twist.h"

#include "gazebo_ros_battery/SetLoad.h"

namespace gazebo
{
class GAZEBO_VISIBLE CmdVelConsumerPlugin : public ModelPlugin
{
public:
    // Constructor
    CmdVelConsumerPlugin();

    ~CmdVelConsumerPlugin() override;

    // Inherited from ModelPlugin
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Init() override;

    void Reset() override;

    void OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr& _msg);

private:
    void QueueThread();

    double CalculatePower(const geometry_msgs::Twist::ConstPtr& _msg);

    // Connection to the World Update events.
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
    common::BatteryPtr battery;

    // Consumer identifier
    int32_t consumerId;

protected:
    double powerLoad;

    // This node is for ros communications
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_power_pub;

    boost::mutex lock;

private:
    ros::CallbackQueue rosQueue;

    std::thread rosQueueThread;
};
}
