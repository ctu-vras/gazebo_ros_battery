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

    std::string robotNamespace;
    if (_sdf->HasElement("robotNamespace"))
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    const auto linkName = _sdf->Get<std::string>("link_name");
    const auto batteryName = _sdf->Get<std::string>("battery_name");
    this->efficiency = _sdf->Get<double>("efficiency", this->efficiency).first;
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power", this->consumerIdlePower).first;
    if (this->efficiency < 0 || this->efficiency > 1)
    {
        gzerr << "efficiency must be between 0 and 1 (inclusive).\n";
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
        gzerr << "Cannot find link with name '" << linkName << "'.\n";
        return;
    }
    this->battery = this->link->Battery(batteryName);
    if (!this->battery)
    {
        gzerr << "Cannot find a battery '" << batteryName << "' in link '" << linkName << "'. Make sure the "
              << "battery_name specified in the plugin can be found in the specified link.\n";
        return;
    }

    if (_sdf->HasElement("joint"))
    {
        auto joint = _sdf->GetElement("joint");
        while (joint)
        {
            const auto jointName = joint->Get<std::string>();
            const auto simJoint = _model->GetJoint(jointName);
            if (simJoint == nullptr)
            {
                gzerr << "<joint> '" << jointName << "' does not exist." << std::endl;
            }
            else
            {
                this->joints[jointName] = simJoint;
            }
            joint = joint->GetNextElement("joint");
        }
    }

    const auto pluginName = _sdf->GetAttribute("name")->GetAsString();
    const auto defaultConsumerName = (this->joints.empty() ? pluginName : this->joints.begin()->first) + "/motor_power";
    const auto consumerName = _sdf->Get<std::string>("consumer_name", defaultConsumerName).first;

    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);
    this->motor_power_pub = this->rosNode->advertise<std_msgs::Float64>(consumerName, 1);
    this->joint_state_sub = this->rosNode->subscribe("joint_states", 1, &MotorConsumerPlugin::OnJointStateMsg, this);

    std::string textJoints = "all joints";
    if (this->joints.size() == 1)
    {
        textJoints = "joint " + this->joints.begin()->first;
    }
    else if (this->joints.size() > 1)
    {
        textJoints = "joints ";
        size_t i = 0;
        for (const auto& nameAndJoint : this->joints)
        {
            textJoints += nameAndJoint.first;
            if (i < this->joints.size() - 1)
                textJoints += ",";
            ++i;
        }
    }
    gzmsg << "Added motor consumer to battery '" << linkName << "/" << batteryName << "' that handles " << textJoints
          << " on topic " << this->rosNode->resolveName(this->joint_state_sub.getTopic()) << ".\n";
}

double MotorConsumerPlugin::CalculatePower(const sensor_msgs::JointState::ConstPtr& _msg)
{
    double totalPower {0.0};
    size_t jointsFound {0u};

    for (size_t i = 0; i < _msg->name.size(); ++i)
    {
        if (!this->joints.empty() && this->joints.find(_msg->name[i]) == this->joints.end())
            continue;
        jointsFound += 1;
        const auto effort = _msg->effort.size() > i ? _msg->effort[i] : 1.0;
        const auto power = std::abs(_msg->velocity[i]) * std::abs(effort) / this->efficiency;
        totalPower += power;
    }

    if (jointsFound == 0)
        return -1;

#ifdef MOTOR_CONSUMER_DEBUG
    gzdbg << "motor_consumer: " << jointsFound << " joints found. Joint power:" << totalPower << "\n";
#endif
    return std::max(totalPower, this->consumerIdlePower);
}

void MotorConsumerPlugin::OnJointStateMsg(const sensor_msgs::JointState::ConstPtr& _msg)
{
    double motor_power = CalculatePower(_msg);
    if (motor_power == -1)
        return;
    this->battery->SetPowerLoad(this->consumerId, motor_power);
    std_msgs::Float64 motor_power_msg;
    motor_power_msg.data = motor_power;
    this->motor_power_pub.publish(motor_power_msg);
}
