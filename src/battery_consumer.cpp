// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - renamed a few SDF parameters
// - changed the SetLoad service to a topic
// - extracted base class BatteryConsumerBase

#include "battery_consumer.hh"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

// #define CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() = default;

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    BatteryConsumerBase::Load(_model, _sdf);

    this->powerLoad = this->initialPowerLoad = _sdf->Get<double>("power_load");
    this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

    std_msgs::Float64 powerLoadMsg;
    powerLoadMsg.data = this->powerLoad;
    this->powerLoadPub.publish(powerLoadMsg);

    this->power_load_sub = this->consumerNode->subscribe(
        "power_load_cmd", 1, &BatteryConsumerPlugin::OnPowerLoadCmd, this);

    gzmsg << "Added constant consumer '" << this->consumerName << "' to battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' with power load "
          << this->powerLoad << " W.\n";
}

void BatteryConsumerPlugin::Reset()
{
    std_msgs::Float64 msg;
    msg.data = this->initialPowerLoad;
    this->OnPowerLoadCmd(msg);
    gzdbg << "Battery consumer '" << this->consumerName << "' on battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' was reset.\n";
}

void BatteryConsumerPlugin::OnPowerLoadCmd(const std_msgs::Float64& _msg)
{
    const auto load = this->powerLoad;
    this->powerLoad = _msg.data;
    this->battery->SetPowerLoad(this->consumerId, _msg.data);
    this->powerLoadPub.publish(_msg);

#ifdef CONSUMER_DEBUG
    gzdbg << "Power load of consumer has changed from:" << load << ", to:" << req.power_load << "\n";
#endif
}
