#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_linear_battery
// - cleaned up the code

#include <thread>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_ros_linear_battery/SetLoad.h"

#define CMD_VEL_CONSUMER_DEBUG

namespace gazebo
{
    class GAZEBO_VISIBLE CmdVelConsumerPlugin : public ModelPlugin
    {
    // Constructor
    public: CmdVelConsumerPlugin();

    public: ~CmdVelConsumerPlugin();

    // Inherited from ModelPlugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    public: virtual void Reset();

    public: void OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr &_msg);
    
    private: void QueueThread();

    private: double CalculatePower(const geometry_msgs::Twist::ConstPtr &_msg);

    // Connection to the World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: physics::WorldPtr world;

    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;

    protected: physics::LinkPtr link;

    protected: sdf::ElementPtr sdf;

    // Consumer parameter
    protected: double powerLoadRate;
    protected: double consumerIdlePower;

    // Battery
    private: common::BatteryPtr battery;

    // Consumer identifier
    private: int32_t consumerId;

    protected: double powerLoad;

    // This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    protected: ros::Subscriber cmd_vel_sub;
    protected: ros::Publisher cmd_vel_power_pub;

    protected: boost::mutex lock;
    
    private: ros::CallbackQueue rosQueue;
    
    private: std::thread rosQueueThread;
    };

}
