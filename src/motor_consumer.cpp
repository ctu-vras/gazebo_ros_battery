// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include "motor_consumer.hh"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

// #define MOTOR_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorConsumerPlugin);

MotorConsumerPlugin::MotorConsumerPlugin() = default;

MotorConsumerPlugin::~MotorConsumerPlugin()
{
    if (this->battery && this->consumerId != std::numeric_limits<uint32_t>::max())
        this->battery->RemoveConsumer(this->consumerId);
    if (this->rosNode)
        this->rosNode->shutdown();
}

void MotorConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("motor_consumer", "A ROS node for Gazebo has not been initialized, "
                                                 "unable to load plugin. Load the Gazebo system plugin "
                                                 "'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    const auto linkName = _sdf->Get<std::string>("link_name");
    const auto batteryName = _sdf->Get<std::string>("battery_name");
    this->powerLoadRate = _sdf->Get<double>("power_load_rate");
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power");
    if (this->powerLoadRate < 0)
    {
        gzerr << "power_load_rate cannot be negative.\n";
        return;
    }
    if (this->consumerIdlePower < 0)
    {
        gzerr << "consumer_idle_power cannot be negative.\n";
        return;
    }

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

    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);

    this->rosNode = std::make_unique<ros::NodeHandle>(_sdf->Get<std::string>("ros_node"));
    this->motor_power_pub = this->rosNode->advertise<std_msgs::Float64>(
        "/mobile_base/commands/consumer/motor_power", 1);
    this->joint_state_sub = this->rosNode->subscribe(
        "/" + this->model->GetName() + "/joint_states", 1, &MotorConsumerPlugin::OnJointStateMsg, this);

    gzlog << "motor consumer loaded\n";
}

double MotorConsumerPlugin::CalculatePower(const sensor_msgs::JointState::ConstPtr& _msg)
{
    double wheelVel = 0;
    for (double velocity : _msg->velocity)
        wheelVel += std::fabs(velocity);
#ifdef MOTOR_CONSUMER_DEBUG
    gzdbg << "motor_consumer: " << _msg->velocity.size() << " joints found. Joint velocity:" << wheelVel << "\n";
#endif
    return std::max(wheelVel * this->powerLoadRate, this->consumerIdlePower);
}

void MotorConsumerPlugin::OnJointStateMsg(const sensor_msgs::JointState::ConstPtr& _msg)
{
    double motor_power = CalculatePower(_msg);
    this->battery->SetPowerLoad(this->consumerId, motor_power);
    std_msgs::Float64 motor_power_msg;
    motor_power_msg.data = motor_power;
    this->motor_power_pub.publish(motor_power_msg);
}
