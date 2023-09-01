// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/msgs/int.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <sdf/sdf.hh>

#include <cras_msgs/PowerStamped.h>
#include <ros/ros.h>

#include <gazebo_ros_battery/battery_consumer_base.hh>

using namespace gazebo;

BatteryConsumerBase::~BatteryConsumerBase()
{
    if (this->battery && this->consumerId != std::numeric_limits<uint32_t>::max())
        this->battery->RemoveConsumer(this->consumerId);
    if (this->rosNode)
        this->rosNode->shutdown();
}

void BatteryConsumerBase::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin "
              << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package.\n";
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();
    this->sdf = _sdf;

    const auto linkName = _sdf->Get<std::string>("link_name");
    this->link = _model->GetLink(linkName);
    if (!this->link)
    {
        gzerr << "Cannot find link with name '" << linkName << "'.\n";
        return;
    }

    const auto batteryName = _sdf->Get<std::string>("battery_name");
    this->battery = this->link->Battery(batteryName);
    if (!this->battery)
    {
        gzerr << "Cannot find battery '" << batteryName << "' in link '" << linkName << "'. Make sure the "
              << "<battery_name> specified in the plugin can be found in the link specified with <link_name>.\n";
        return;
    }

    const auto defaultConsumerName = _sdf->GetAttribute("name")->GetAsString();
    this->consumerName = _sdf->Get<std::string>("consumer_name", defaultConsumerName).first;
    this->consumerId = this->battery->AddConsumer();

    if (_sdf->HasElement("robotNamespace"))
        this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    this->rosNode = std::make_unique<ros::NodeHandle>(this->robotNamespace);
    this->consumerNode = std::make_unique<ros::NodeHandle>(*this->rosNode, this->consumerName);

    this->gzNode.reset(new transport::Node);
    this->gzNode->Init(_model->GetWorld()->Name() + "/" + _model->GetName() + "/" + this->consumerName);

    if (_sdf->Get("publish_ros_topic", true).first)
        this->powerLoadPub = this->consumerNode->advertise<cras_msgs::PowerStamped>("power_load", 1, true);
    this->gzConsumerIdPub = this->gzNode->Advertise<gazebo::msgs::Int>("~/consumer_id", 1);
    this->gzPowerLoadPub = this->gzNode->Advertise<gazebo::msgs::Any>("~/power_load", 1);

    this->enabled = _sdf->Get("enabled", true).first;
    this->gzEnableSub = this->gzNode->Subscribe("~/enable", &BatteryConsumerBase::OnGzEnabledMsg, this, true);

    gazebo::msgs::Int consumerIdMsg;
    consumerIdMsg.set_data(static_cast<int32_t>(this->consumerId));
    this->gzConsumerIdPub->Publish(consumerIdMsg);  // Make sure all subscribers are latching
}

void BatteryConsumerBase::Publish(const double powerLoad, const gazebo::common::Time& time,
                                  const ros::Duration& sampleInterval, const double variance)
{
    this->gzPowerLoadPub->Publish(gazebo::msgs::ConvertAny(powerLoad));

    if (this->powerLoadPub)
    {
        cras_msgs::PowerStamped powerLoadMsg;
        powerLoadMsg.header.frame_id = this->consumerName;
        powerLoadMsg.header.stamp.sec = time.sec;
        powerLoadMsg.header.stamp.nsec = time.nsec;
        powerLoadMsg.measurement.data.power = powerLoad;
        powerLoadMsg.measurement.sample_duration = sampleInterval;
        powerLoadMsg.measurement.variance = variance;
        this->powerLoadPub.publish(powerLoadMsg);
    }
}

void BatteryConsumerBase::Publish(const double powerLoad, const gazebo::common::Time& time,
                                  const ros::Duration& sampleInterval)
{
    this->Publish(powerLoad, time, sampleInterval, 0.0);
}

void BatteryConsumerBase::Publish(const double powerLoad, const gazebo::common::Time& time,
                                  const double variance)
{
    this->Publish(powerLoad, time, {}, variance);
}

void BatteryConsumerBase::Publish(const double powerLoad, const gazebo::common::Time& time)
{
    this->Publish(powerLoad, time, {}, 0.0);
}

void BatteryConsumerBase::Publish(const double powerLoad)
{
    this->Publish(powerLoad, this->world->SimTime(), {}, 0.0);
}

void BatteryConsumerBase::Reset()
{
    this->enabled = this->sdf->Get("enabled", true).first;
}

void BatteryConsumerBase::OnGzEnabledMsg(const ConstAnyPtr& msg)
{
    if (msg->has_type() && msg->type() == msgs::Any_ValueType_BOOLEAN && msg->has_bool_value())
        this->SetEnabled(msg->bool_value());
}

void BatteryConsumerBase::SetEnabled(const bool enabled)
{
    if (this->enabled == enabled)
        return;

    this->enabled = enabled;

    if (!enabled && this->battery != nullptr && this->consumerId != std::numeric_limits<uint32_t>::max())
    {
        this->battery->SetPowerLoad(this->consumerId, 0);
        this->Publish(0);
    }
}
