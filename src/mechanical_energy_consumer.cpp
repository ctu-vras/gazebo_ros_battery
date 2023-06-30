// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "mechanical_energy_consumer.hh"

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

// #define MECHANICAL_ENERGY_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MechanicalEnergyConsumerPlugin);

MechanicalEnergyConsumerPlugin::MechanicalEnergyConsumerPlugin() = default;

MechanicalEnergyConsumerPlugin::~MechanicalEnergyConsumerPlugin()
{
    if (this->battery && this->consumerId != std::numeric_limits<uint32_t>::max())
        this->battery->RemoveConsumer(this->consumerId);
    if (this->rosNode)
        this->rosNode->shutdown();
}

void MechanicalEnergyConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mechanical_energy_consumer", "A ROS node for Gazebo has not been initialized, "
                                                             "unable to load plugin. Load the Gazebo system plugin "
                                                             "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                                                             "package.");
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
    this->friction = _sdf->Get<double>("friction", this->friction).first;
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power", this->consumerIdlePower).first;
    if (this->friction < 0)
    {
        gzerr << "friction cannot be negative.\n";
        return;
    }
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

    this->publishInterval = _sdf->Get<double>("publish_interval", this->publishInterval).first;

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

    const auto pluginName = _sdf->GetAttribute("name")->GetAsString();
    const auto defaultConsumerName = pluginName + "/mechanical_energy_power";
    const auto consumerName = _sdf->Get<std::string>("consumer_name", defaultConsumerName).first;

    this->rosNode = std::make_unique<ros::NodeHandle>(robotNamespace);
    this->power_pub = this->rosNode->advertise<std_msgs::Float64>(consumerName, 1);

    this->beforePhysicsUpdateConnection = event::Events::ConnectBeforePhysicsUpdate(
        std::bind(&MechanicalEnergyConsumerPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "Mechanical energy consumer loaded\n";
}

void MechanicalEnergyConsumerPlugin::OnUpdate(const common::UpdateInfo& info)
{
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

    this->consumedCharge += totalPower * dt;

    if (this->lastPublishTime + this->publishInterval <= info.simTime)
    {
        std_msgs::Float64 power_msg;
        power_msg.data = this->consumedCharge / (info.simTime - this->lastPublishTime).Double();
        this->power_pub.publish(power_msg);
        this->lastPublishTime = info.simTime;
        this->consumedCharge = 0.0;
    }
}
