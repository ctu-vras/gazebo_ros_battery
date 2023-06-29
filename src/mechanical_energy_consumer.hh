#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>
#include <memory>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

class GAZEBO_VISIBLE MechanicalEnergyConsumerPlugin : public ModelPlugin
{
public:
    MechanicalEnergyConsumerPlugin();

    ~MechanicalEnergyConsumerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void OnUpdate(const common::UpdateInfo& _info);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;

    double efficiency {1.0};  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.
    double friction {0.0};  //!< Fraction of kinetic energy converted to friction energy.
    double publishInterval {1.0};  //!< How often the power output should be published to a ROS topic

private:
    common::BatteryPtr battery;

    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier.

    common::Time lastPublishTime;
    double lastEnergy {-1.0};
    common::Time lastUpdateTime;
    double consumedCharge {0.0};

protected:
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher power_pub;
    event::ConnectionPtr beforePhysicsUpdateConnection;
};
}
