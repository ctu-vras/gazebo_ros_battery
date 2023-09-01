#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/rosin-project/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code
// - changed to publish BatteryState message
// - removed the services
// - added temperature modeling

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

namespace gazebo
{

/// \brief A plugin that simulates a linear battery model: discharge and charge according to linear power models.
class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
{
public:
    BatteryPlugin();

    ~BatteryPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Init() override;

    void Reset() override;

private:
    double OnUpdateVoltage(const common::BatteryPtr& _battery);

    void OnRosAmbientTempMsg(const sensor_msgs::Temperature& _msg);
    void OnGzAmbientTempMsg(const ConstAnyPtr& _msg);
    void OnGzAllowChargingMsg(const ConstAnyPtr& _msg);

    void UpdateResistance();

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    common::BatteryPtr battery;
    sdf::ElementPtr sdf;
    common::Time lastUpdateTime;
    double updatePeriod {1.0};
    bool allowCharging {true};
    bool reportCellVoltage {false};
    bool reportCellTemperature {false};
    bool computeResistance {false};
    bool computeTemperature {false};
    double baseTemperature {std::numeric_limits<double>::quiet_NaN()};
    std::vector<double> resistanceTemperatureCoeffs;
    double heatDissipationRate {0.0};
    double heatCapacity {1.0};
    double ambientTemperature {25.0};

    // E(t) = e0 + e1* Q(t)/c
    double e0 {0.0};
    double e1 {0.0};

    double q0 {0.0};  //!< Initial battery charge in Ah.

    double c {0.0};  //!< Battery capacity in Ah.

    double r {0.0};  //!< Battery inner resistance in Ohms.

    double tau {0.0};  //!< Current low-pass filter characteristic time in seconds.

    double ismooth {0.0};  //!< Smoothed battery current in A.

    double q {0.0};  //!< Instantaneous battery charge in Ah.

    double t {std::numeric_limits<double>::quiet_NaN()};  //!< Temperature of the battery in degrees Celsius.

    double heatEnergy {0.0};  //!< Heat energy stored in the battery in Joules.

    std::unique_ptr<ros::NodeHandle> rosNode;
    gazebo::transport::NodePtr gzNode;

    ros::Publisher battery_state;
    ros::Publisher charge_state_wh;
    ros::Subscriber rosAmbientTemperatureSub;

    gazebo::transport::SubscriberPtr gzAmbientTemperatureSub;
    gazebo::transport::SubscriberPtr gzAllowChargingSub;

    sensor_msgs::BatteryState batteryMsg;
};
}
