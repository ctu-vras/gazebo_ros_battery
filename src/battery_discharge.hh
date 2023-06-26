#ifndef gazebo_ros_linear_battery_BATTERY_DISCHARGE_H
#define gazebo_ros_linear_battery_BATTERY_DISCHARGE_H

#include <map>
#include <string>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include <boost/thread/mutex.hpp>
#include "std_msgs/Bool.h"
#include "gazebo_ros_linear_battery/SetCharge.h"
#include "gazebo_ros_linear_battery/SetCharging.h"
#include "gazebo_ros_linear_battery/SetCoef.h"
#include "gazebo_ros_linear_battery/SetChargingRate.h"

// #define BATTERY_DEBUG
#define DBG_INTERVAL 5.0

namespace gazebo
{
    /// \brief A plugin that simulate BRASS CP1 battery model: discharge and charge according to power models
    class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
    {
    /// \brief Constructor
    public: BatteryPlugin();

    public: ~BatteryPlugin();

    // Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    public: virtual void Reset();

    private: double OnUpdateVoltage(const common::BatteryPtr &_battery);

    public: bool SetCharging(gazebo_ros_linear_battery::SetCharging::Request& req,
                             gazebo_ros_linear_battery::SetCharging::Response& res);

    public: bool SetCharge(gazebo_ros_linear_battery::SetCharge::Request& req,
                             gazebo_ros_linear_battery::SetCharge::Response& res);

    public: bool SetModelCoefficients(gazebo_ros_linear_battery::SetCoef::Request& req,
                                      gazebo_ros_linear_battery::SetCoef::Response& res);

    public: bool SetChargingRate(gazebo_ros_linear_battery::SetChargingRate::Request& req,
                                 gazebo_ros_linear_battery::SetChargingRate::Response& res);

    // Connection to the World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: physics::WorldPtr world;

    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;

    protected: physics::LinkPtr link;

    protected: common::BatteryPtr battery;

    protected: sdf::ElementPtr sdf;


    // E(t) = e0 + e1* Q(t)/c
    protected: double et;
    protected: double e0;
    protected: double e1;

    // Initial battery charge in Ah.
    protected: double q0;

    // Charge rate in A
    // More description about charge rate: http://batteriesbyfisher.com/determining-charge-time
    protected: double qt;

    // Battery capacity in Ah.
    protected: double c;

    // Battery inner resistance in Ohm
    protected: double r;

    // Current low-pass filter characteristic time in seconds.
    protected: double tau;

    // Raw battery current in A.
    protected: double iraw;

    // Smoothed battery current in A.
    protected: double ismooth;

    // Instantaneous battery charge in Ah.
    protected: double q;

    // This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    protected: ros::Publisher charge_state;
    protected: ros::Publisher charge_state_mwh;
    protected: ros::Publisher motor_power;

    protected: ros::ServiceServer set_charging;
    protected: ros::ServiceServer set_charge;
    protected: ros::ServiceServer set_coefficients;
    protected: ros::ServiceServer set_charging_rate;

    protected: boost::mutex lock;

    protected: bool charging;

    protected: double sim_time_now;

    };
}

#endif //gazebo_ros_linear_battery_BATTERY_DISCHARGE_H