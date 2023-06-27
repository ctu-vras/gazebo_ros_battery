// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include "battery_consumer.hh"

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

// #define CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() = default;

BatteryConsumerPlugin::~BatteryConsumerPlugin()
{
    if (this->battery && this->consumerId != std::numeric_limits<uint32_t>::max())
        this->battery->RemoveConsumer(this->consumerId);
    if (this->rosNode)
        this->rosNode->shutdown();
}

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("battery_consumer", "A ROS node for Gazebo has not been initialized, "
                                                   "unable to load plugin. Load the Gazebo system plugin "
                                                   "'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    const auto linkName = _sdf->Get<std::string>("link_name");
    const auto batteryName = _sdf->Get<std::string>("battery_name");

    this->link = _model->GetLink(linkName);
    if (!this->link)
    {
        gzerr << "Cannot find a link with name '" << linkName << "'.\n";
        return;
    }

    this->battery = this->link->Battery(batteryName);
    if (!this->battery)
    {
        gzerr << "Cannot find a battery '" << batteryName << "' in link '" << linkName << "'. Make sure the "
              << "battery_name specified in the plugin can be found in the specified link.\n";
        return;
    }

    // Add consumer and sets its power load
    this->powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, powerLoad);

    // Create ros node and publish stuff there!
    this->rosNode = std::make_unique<ros::NodeHandle>(_sdf->Get<std::string>("ros_node"));

    this->set_power_load = this->rosNode->advertiseService(
        this->model->GetName() + "/set_power_load", &BatteryConsumerPlugin::SetConsumerPowerLoad, this);

    gzlog << "consumer loaded\n";
}

bool BatteryConsumerPlugin::SetConsumerPowerLoad(gazebo_ros_battery::SetLoad::Request& req,
                                                 gazebo_ros_battery::SetLoad::Response& res)
{
    const auto load = this->powerLoad;
    this->powerLoad = req.power_load;
    this->battery->SetPowerLoad(this->consumerId, req.power_load);

#ifdef CONSUMER_DEBUG
    gzdbg << "Power load of consumer has changed from:" << load << ", to:" << req.power_load << "\n";
#endif

    res.result = true;
    return true;
}
