// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include "cmd_vel_consumer.hh"

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// #define CMD_VEL_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CmdVelConsumerPlugin);

CmdVelConsumerPlugin::CmdVelConsumerPlugin() = default;

CmdVelConsumerPlugin::~CmdVelConsumerPlugin()
{
    if (this->battery && this->consumerId != std::numeric_limits<uint32_t>::max())
        this->battery->RemoveConsumer(this->consumerId);
    if (this->rosNode)
        this->rosNode->shutdown();
}

void CmdVelConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("cmd_vel_consumer", "A ROS node for Gazebo has not been initialized, "
                                                   "unable to load plugin. Load the Gazebo system plugin "
                                                   "'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    const auto linkName = _sdf->Get<std::string>("link_name");
    const auto batteryName = _sdf->Get<std::string>("battery_name");
    const auto cmdVelTopic = _sdf->Get<std::string>("cmd_vel_topic");
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
    this->cmd_vel_power_pub = this->rosNode->advertise<std_msgs::Float64>(
        "/mobile_base/commands/consumer/cmd_vel_power", 1);
    this->cmd_vel_sub = this->rosNode->subscribe(
        "/" + this->model->GetName() + cmdVelTopic, 1, &CmdVelConsumerPlugin::OnCmdVelMsg, this);

    gzlog << "cmd_vel consumer loaded\n";
}

double CmdVelConsumerPlugin::CalculatePower(const geometry_msgs::Twist::ConstPtr& _msg)
{
    double linearSpeed = _msg->linear.x + _msg->linear.y + _msg->linear.z;
    double angularSpeed = _msg->angular.z;
    return 5;
}

void CmdVelConsumerPlugin::OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr& _msg)
{
    const auto cmd_vel_power = CalculatePower(_msg);
    this->battery->SetPowerLoad(this->consumerId, cmd_vel_power);
    std_msgs::Float64 cmd_vel_power_msg;
    cmd_vel_power_msg.data = cmd_vel_power;
    this->cmd_vel_power_pub.publish(cmd_vel_power_msg);
}
