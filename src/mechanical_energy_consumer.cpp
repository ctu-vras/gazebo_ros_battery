// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "mechanical_energy_consumer.hh"

#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <cras_msgs/PowerStamped.h>
#include <ros/ros.h>

// #define MECHANICAL_ENERGY_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MechanicalEnergyConsumerPlugin);

MechanicalEnergyConsumerPlugin::MechanicalEnergyConsumerPlugin() = default;

void MechanicalEnergyConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    BatteryConsumerBase::Load(_model, _sdf);

    this->efficiency = _sdf->Get<double>("efficiency", this->efficiency).first;
    if (this->efficiency < 0 || this->efficiency > 1)
    {
        gzerr << "efficiency must be between 0 and 1 (inclusive).\n";
        return;
    }

    this->friction = _sdf->Get<double>("friction", this->friction).first;
    if (this->friction < 0)
    {
        gzerr << "friction cannot be negative.\n";
        return;
    }

    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power", this->consumerIdlePower).first;
    if (this->consumerIdlePower < 0)
    {
        gzerr << "consumer_idle_power cannot be negative.\n";
        return;
    }

    this->ignoreFirstDuration = _sdf->Get<double>("ignore_first_duration", this->ignoreFirstDuration).first;
    if (this->ignoreFirstDuration < 0)
    {
        gzerr << "ignore_first_duration cannot be negative.\n";
        return;
    }

    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);

    this->beforePhysicsUpdateConnection = event::Events::ConnectBeforePhysicsUpdate(
        std::bind(&MechanicalEnergyConsumerPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "Added mechanical energy consumer '" << this->consumerName << "' to battery '"
          << this->link->GetName() << "/" << this->battery->Name() << ".\n";
}

void MechanicalEnergyConsumerPlugin::OnUpdate(const common::UpdateInfo& info)
{
    if (this->firstUpdateTime == common::Time::Zero)
        this->firstUpdateTime = info.simTime;
    if (info.simTime < this->firstUpdateTime + this->ignoreFirstDuration)
        return;
    if (!this->initialized)
    {
        this->lastUpdateTime = this->lastPublishTime = info.simTime;
        this->lastEnergy = this->model->GetWorldEnergyPotential();
        this->initialized = true;
        return;
    }

    const double potentialEnergy = this->model->GetWorldEnergyPotential();
    const double kineticEnergy = this->model->GetWorldEnergyKinetic();
    const auto prevEnergy = (this->lastEnergy >= 0) ? this->lastEnergy : potentialEnergy;
    this->lastEnergy = potentialEnergy + kineticEnergy;
    const auto dt = (info.simTime - this->lastUpdateTime).Double();
    this->lastUpdateTime = info.simTime;

    const auto mechanicalEnergyDiff = std::abs(this->lastEnergy - prevEnergy) / this->efficiency;
    const auto mechanicalEnergyPower = mechanicalEnergyDiff / dt;
    const auto frictionPower = this->friction * kineticEnergy;
    const auto totalPower = mechanicalEnergyPower + frictionPower;

    const auto load = (std::max)(totalPower, this->consumerIdlePower);
    this->battery->SetPowerLoad(this->consumerId, load);

    this->consumedCharge += load * dt;

    if (this->lastPublishTime + this->publishInterval <= info.simTime)
    {
        cras_msgs::PowerStamped power_load_msg;
        power_load_msg.header.frame_id = this->consumerName;
        power_load_msg.header.stamp.sec = info.simTime.sec;
        power_load_msg.header.stamp.nsec = info.simTime.nsec;
        power_load_msg.measurement.sample_duration = ros::Duration(this->publishInterval);
        power_load_msg.measurement.data.power = this->consumedCharge / (info.simTime - this->lastPublishTime).Double();
        this->powerLoadPub.publish(power_load_msg);
        this->lastPublishTime = info.simTime;
        this->consumedCharge = 0.0;
    }
}

void MechanicalEnergyConsumerPlugin::Reset()
{
    this->lastEnergy = -1.0;
    this->lastUpdateTime = this->lastPublishTime = this->firstUpdateTime = common::Time::Zero;
    this->consumedCharge = 0.0;
    this->initialized = false;
    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);
    cras_msgs::PowerStamped power_msg;
    power_msg.header.frame_id = this->consumerName;
    power_msg.header.stamp.sec = this->model->GetWorld()->SimTime().sec;
    power_msg.header.stamp.nsec = this->model->GetWorld()->SimTime().nsec;
    power_msg.measurement.data.power = this->consumerIdlePower;
    this->powerLoadPub.publish(power_msg);
    gzdbg << "Mechanical energy consumer '" << this->consumerName << "' on battery '"
          << this->link->GetName() << "/" << this->battery->Name() << "' was reset.\n";
}
