#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - extracted base class BatteryConsumerBase

#include <limits>
#include <memory>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <cras_msgs/Power.h>
#include <ros/ros.h>

#include <gazebo_ros_battery/battery_consumer_base.hh>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerPlugin : public BatteryConsumerBase
{
public:
    BatteryConsumerPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    void OnPowerLoadCmd(const cras_msgs::Power& _msg);

    void OnGzPowerLoadCmd(const ConstAnyPtr& _msg);

protected:
    void SetEnabled(bool enabled) override;

    double initialPowerLoad {0.0};
    double powerLoad {0.0};

    ros::Subscriber power_load_sub;
    gazebo::transport::SubscriberPtr gz_power_load_sub;
    event::ConnectionPtr updateConnection;
};

}
