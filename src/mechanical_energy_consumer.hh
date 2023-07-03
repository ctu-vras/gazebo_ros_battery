#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <gazebo_ros_battery/battery_consumer_base.hh>

namespace gazebo
{

class GAZEBO_VISIBLE MechanicalEnergyConsumerPlugin : public BatteryConsumerBase
{
public:
    MechanicalEnergyConsumerPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    void OnUpdate(const common::UpdateInfo& _info);

protected:
    double efficiency {1.0};  //!< Consumer parameter.
    double consumerIdlePower {0.0};  //!< Consumer parameter.
    double friction {0.0};  //!< Fraction of kinetic energy converted to friction energy.
    double publishInterval {1.0};  //!< How often the power output should be published to a ROS topic
    double ignoreFirstDuration {1.0};  //!< How many seconds since first Update() call should this consumer be ignored.

    common::Time lastPublishTime;
    double lastEnergy {-1.0};
    common::Time firstUpdateTime;
    common::Time lastUpdateTime;
    double consumedCharge {0.0};
    bool initialized {false};

    ros::Publisher power_pub;
    event::ConnectionPtr beforePhysicsUpdateConnection;
};
}
