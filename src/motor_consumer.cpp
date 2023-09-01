// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - extracted base class BatteryConsumerBase

#include "motor_consumer.hh"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <cras_msgs/PowerStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// #define MOTOR_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorConsumerPlugin);

MotorConsumerPlugin::MotorConsumerPlugin() = default;

void MotorConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    BatteryConsumerBase::Load(_model, _sdf);

    this->efficiency = _sdf->Get<double>("efficiency", this->efficiency).first;
    if (this->efficiency < 0 || this->efficiency > 1)
    {
        gzerr << "efficiency must be between 0 and 1 (inclusive).\n";
        return;
    }

    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power", this->consumerIdlePower).first;
    if (this->consumerIdlePower < 0)
    {
        gzerr << "consumer_idle_power cannot be negative.\n";
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

    if (this->enabled)
    {
        this->lastPowerLoad = this->consumerIdlePower;
        this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);
        this->Publish(this->consumerIdlePower);
    }

    const auto jointStatesTopic = _sdf->Get<std::string>("joint_states_topic", "joint_states").first;

    this->joint_state_sub = this->rosNode->subscribe(jointStatesTopic, 1, &MotorConsumerPlugin::OnJointStateMsg, this);

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
    gzmsg << "Added motor consumer '" << this->consumerName << "' to battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' that handles " << textJoints
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
    this->lastPowerLoad = motor_power;

    if (this->enabled)
    {
        this->battery->SetPowerLoad(this->consumerId, motor_power);
        this->Publish(motor_power);
    }
}

void MotorConsumerPlugin::Reset()
{
    BatteryConsumerBase::Reset();

    this->battery->SetPowerLoad(this->consumerId, this->enabled ? this->consumerIdlePower : 0);
    this->Publish(this->enabled ? this->consumerIdlePower : 0);

    gzdbg << "Motor consumer '" << this->consumerName << "' on battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' was reset.\n";
}

void MotorConsumerPlugin::SetEnabled(bool enabled)
{
    if (!this->enabled && enabled)
    {
        if (this->battery != nullptr && this->consumerId != std::numeric_limits<uint32_t>::max())
        {
            this->battery->SetPowerLoad(this->consumerId, this->lastPowerLoad);
            this->Publish(this->lastPowerLoad);
        }
    }
    else
    {
        BatteryConsumerBase::SetEnabled(enabled);
    }
}
