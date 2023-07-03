#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/Battery.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

#include <sdf/Element.hh>

#include <ros/node_handle.h>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerBase : public gazebo::ModelPlugin
{
public:
    ~BatteryConsumerBase() override;
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;
    common::BatteryPtr battery;

    std::string robotNamespace;
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::unique_ptr<ros::NodeHandle> consumerNode;
    ros::Publisher powerLoadPub;

    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier
    std::string consumerName;
};

}
