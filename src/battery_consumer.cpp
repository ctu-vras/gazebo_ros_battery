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

#include <cras_msgs/Power.h>
#include <cras_msgs/PowerStamped.h>
#include <ros/ros.h>

// #define CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() = default;

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    BatteryConsumerBase::Load(_model, _sdf);

    this->powerLoad = this->initialPowerLoad = _sdf->Get<double>("power_load");
    this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

    cras_msgs::PowerStamped powerLoadMsg;
    powerLoadMsg.header.frame_id = this->consumerName;
    powerLoadMsg.header.stamp.sec = _model->GetWorld()->SimTime().sec;
    powerLoadMsg.header.stamp.nsec = _model->GetWorld()->SimTime().nsec;
    powerLoadMsg.measurement.data.power = this->powerLoad;
    this->powerLoadPub.publish(powerLoadMsg);

    this->power_load_sub = this->consumerNode->subscribe(
        "power_load_cmd", 1, &BatteryConsumerPlugin::OnPowerLoadCmd, this);

    gzmsg << "Added constant consumer '" << this->consumerName << "' to battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' with power load "
          << this->powerLoad << " W.\n";
}

void BatteryConsumerPlugin::Reset()
{
    cras_msgs::Power powerLoadMsg;
    powerLoadMsg.power = this->initialPowerLoad;
    this->OnPowerLoadCmd(powerLoadMsg);
    gzdbg << "Battery consumer '" << this->consumerName << "' on battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' was reset.\n";
}

void BatteryConsumerPlugin::OnPowerLoadCmd(const cras_msgs::Power& _msg)
{
    const auto load = this->powerLoad;
    this->powerLoad = _msg.power;
    this->battery->SetPowerLoad(this->consumerId, _msg.power);

    cras_msgs::PowerStamped powerLoadMsg;
    powerLoadMsg.header.frame_id = this->consumerName;
    powerLoadMsg.header.stamp.sec = this->model->GetWorld()->SimTime().sec;
    powerLoadMsg.header.stamp.nsec = this->model->GetWorld()->SimTime().nsec;
    powerLoadMsg.measurement.data = _msg;
    this->powerLoadPub.publish(powerLoadMsg);

#ifdef CONSUMER_DEBUG
    gzdbg << "Power load of consumer has changed from:" << load << ", to:" << req.power_load << "\n";
#endif
}
