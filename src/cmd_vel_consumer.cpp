// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - added the ability to subscribe also to Gazebo topics
// - reworked the total power computation

#include "cmd_vel_consumer.hh"

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/msgs/twist.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
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

    auto robotNamespace = _model->GetName();
    if (_sdf->HasElement("robotNamespace"))
    {
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
        if (robotNamespace.empty()) robotNamespace = _model->GetName();
    }
    if (!robotNamespace.empty()) robotNamespace += "/";

    const auto linkName = _sdf->Get<std::string>("link_name");
    const auto batteryName = _sdf->Get<std::string>("battery_name");

    this->powerLoadRates.mutable_linear()->set_x(_sdf->Get<double>("power_load_rate_x", 0.0).first);
    this->powerLoadRates.mutable_linear()->set_y(_sdf->Get<double>("power_load_rate_y", 0.0).first);
    this->powerLoadRates.mutable_linear()->set_z(_sdf->Get<double>("power_load_rate_z", 0.0).first);
    this->powerLoadRates.mutable_angular()->set_x(_sdf->Get<double>("power_load_rate_roll", 0.0).first);
    this->powerLoadRates.mutable_angular()->set_y(_sdf->Get<double>("power_load_rate_pitch", 0.0).first);
    this->powerLoadRates.mutable_angular()->set_z(_sdf->Get<double>("power_load_rate_yaw", 0.0).first);
    
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power");
    if (this->consumerIdlePower < 0)
    {
        gzerr << "consumer_idle_power cannot be negative.\n";
        return;
    }

    this->commandDuration = _sdf->Get<double>("command_duration", this->commandDuration).first;
    
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

    const auto gz_pose_topic = _sdf->Get<std::string>("gz_pose_topic", "").first;
    const auto gz_twist_topic = _sdf->Get<std::string>("gz_twist_topic", "").first;

    if (!gz_pose_topic.empty() || !gz_twist_topic.empty())
    {
        this->gzNode.reset(new transport::Node);
        this->gzNode->Init();
    }

    if (!gz_pose_topic.empty())
        this->gz_pose_sub = this->gzNode->Subscribe<msgs::Pose>(
            gz_pose_topic, &CmdVelConsumerPlugin::OnGzPoseMsg, this);
    if (!gz_twist_topic.empty())
        this->gz_twist_sub = this->gzNode->Subscribe<msgs::Twist>(
            gz_twist_topic, &CmdVelConsumerPlugin::OnGzTwistMsg, this);

    const auto pluginName = _sdf->GetAttribute("name")->GetAsString();
    const auto defaultConsumerName = pluginName + "/cmd_vel_power";
    const auto consumerName = _sdf->Get<std::string>("consumer_name", defaultConsumerName).first;

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);
    this->cmd_vel_power_pub = this->rosNode->advertise<std_msgs::Float64>(consumerName, 1);

    const auto cmdVelTopic = _sdf->Get<std::string>("ros_cmd_vel_topic", "").first;
    if (!cmdVelTopic.empty())
        this->cmd_vel_sub = this->rosNode->subscribe(cmdVelTopic, 1, &CmdVelConsumerPlugin::OnCmdVelMsg, this);

    this->beforePhysicsUpdateConnection = event::Events::ConnectBeforePhysicsUpdate(
        std::bind(&CmdVelConsumerPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "cmd_vel consumer loaded\n";
}

double CmdVelConsumerPlugin::CalculatePower(const geometry_msgs::Twist& _msg)
{
    double totalPower = 0.0;
    totalPower += std::abs(_msg.linear.x) * this->powerLoadRates.linear().x();
    totalPower += std::abs(_msg.linear.y) * this->powerLoadRates.linear().y();
    totalPower += std::abs(_msg.linear.z) * this->powerLoadRates.linear().z();
    totalPower += std::abs(_msg.angular.x) * this->powerLoadRates.angular().x();
    totalPower += std::abs(_msg.angular.y) * this->powerLoadRates.angular().y();
    totalPower += std::abs(_msg.angular.z) * this->powerLoadRates.angular().z();
    return (std::max)(totalPower, this->consumerIdlePower);
}

void CmdVelConsumerPlugin::OnGzTwistMsg(const ConstTwistPtr& _msg)
{
    geometry_msgs::Twist rosMsg;
    rosMsg.linear.x = _msg->linear().x();
    rosMsg.linear.y = _msg->linear().y();
    rosMsg.linear.z = _msg->linear().z();
    rosMsg.angular.x = _msg->angular().x();
    rosMsg.angular.y = _msg->angular().y();
    rosMsg.angular.z = _msg->angular().z();
    this->OnCmdVelMsg(rosMsg);
}

void CmdVelConsumerPlugin::OnGzPoseMsg(const ConstPosePtr& _msg)
{
    geometry_msgs::Twist rosMsg;
    rosMsg.linear.x = _msg->position().x();
    rosMsg.linear.y = _msg->position().y();
    rosMsg.linear.z = _msg->position().z();
    const auto euler = msgs::ConvertIgn(_msg->orientation()).Euler();
    rosMsg.angular.x = euler.X();
    rosMsg.angular.y = euler.Y();
    rosMsg.angular.z = euler.Z();
    this->OnCmdVelMsg(rosMsg);
}

void CmdVelConsumerPlugin::OnCmdVelMsg(const geometry_msgs::Twist& _msg)
{
    this->lastCmdTime = this->world->SimTime();
    const auto cmd_vel_power = CalculatePower(_msg);
    this->battery->SetPowerLoad(this->consumerId, cmd_vel_power);
    std_msgs::Float64 cmd_vel_power_msg;
    cmd_vel_power_msg.data = cmd_vel_power;
    this->cmd_vel_power_pub.publish(cmd_vel_power_msg);
}

void CmdVelConsumerPlugin::OnUpdate(const common::UpdateInfo&)
{
    // Zero out the command if none was recently received
    if (this->lastCmdTime + this->commandDuration < this->world->SimTime())
        this->OnCmdVelMsg(geometry_msgs::Twist());
}
