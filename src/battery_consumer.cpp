// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - renamed a few SDF parameters

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

    auto robotNamespace = _model->GetName();
    if (_sdf->HasElement("robotNamespace"))
    {
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
        if (robotNamespace.empty()) robotNamespace = _model->GetName();
    }
    if (!robotNamespace.empty()) robotNamespace += "/";

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

    const auto defaultConsumerName = _sdf->GetAttribute("name")->GetAsString();
    const auto consumerName = _sdf->Get<std::string>("consumer_name", defaultConsumerName).first;

    this->powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, powerLoad);

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);

    this->set_power_load = ros::NodeHandle(*this->rosNode, consumerName).advertiseService(
        "set_power_load", &BatteryConsumerPlugin::SetConsumerPowerLoad, this);

    gzmsg << "Added constant consumer to battery '" << linkName << "/" << batteryName << "' with power load "
          << this->powerLoad << " W.\n";
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
