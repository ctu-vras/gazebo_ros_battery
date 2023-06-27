#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include <boost/thread/mutex.hpp>
#include "gazebo_ros_battery/SetLoad.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerPlugin : public ModelPlugin
{
    // Constructor
public:
    BatteryConsumerPlugin();

    ~BatteryConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Init() override;

    void Reset() override;

    bool SetConsumerPowerLoad(gazebo_ros_battery::SetLoad::Request& req,
                              gazebo_ros_battery::SetLoad::Response& res);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;
    common::BatteryPtr battery;

private:
    // Consumer identifier
    int32_t consumerId;

protected:
    double powerLoad;

    // This node is for ros communications
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::ServiceServer set_power_load;

    boost::mutex lock;
};
}
