#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_linear_battery
// - cleaned up the code

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include <boost/thread/mutex.hpp>
#include "gazebo_ros_linear_battery/SetLoad.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

// #define CONSUMER_DEBUG

namespace gazebo
{
    class GAZEBO_VISIBLE BatteryConsumerPlugin : public ModelPlugin
    {
    // Constructor
    public: BatteryConsumerPlugin();

    public: ~BatteryConsumerPlugin();

    // Inherited from ModelPlugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    public: virtual void Reset();

    public: bool SetConsumerPowerLoad(gazebo_ros_linear_battery::SetLoad::Request& req,
                                      gazebo_ros_linear_battery::SetLoad::Response& res);

    // Connection to the World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: physics::WorldPtr world;

    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;

    protected: physics::LinkPtr link;

    protected: sdf::ElementPtr sdf;

    // Battery
    private: common::BatteryPtr battery;

    // Consumer identifier
    private: int32_t consumerId;

    protected: double powerLoad;

    // This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    protected: ros::ServiceServer set_power_load;

    protected: boost::mutex lock;

    };

}
