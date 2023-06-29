#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <limits>
#include <memory>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerPlugin : public ModelPlugin
{
public:
    BatteryConsumerPlugin();

    ~BatteryConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void OnPowerLoadCmd(const std_msgs::Float64& _msg);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;
    common::BatteryPtr battery;

private:
    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier

protected:
    double powerLoad {0.0};

    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber power_load_sub;
};
}
