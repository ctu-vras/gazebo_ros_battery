// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
// - renamed to gazebo_ros_battery
// - cleaned up the code

#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"

#include "motor_consumer.hh"


#define MOTOR_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorConsumerPlugin);

MotorConsumerPlugin::MotorConsumerPlugin() : consumerId(-1)
{
}

MotorConsumerPlugin::~MotorConsumerPlugin()
{
    if (this->battery && this->consumerId !=-1)
        this->battery->RemoveConsumer(this->consumerId);
}

void MotorConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM_NAMED("motor_consumer", "A ROS node for Gazebo has not been initialized, "
            "unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
            "gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string linkName = _sdf->Get<std::string>("link_name");
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    this->powerLoadRate = _sdf->Get<double>("power_load_rate");
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power");
    GZ_ASSERT(this->powerLoadRate >= 0, "consume_rate cannot be negative.");
    GZ_ASSERT(this->consumerIdlePower >= 0, "consume_constant cannot be negative.");

    this->link = _model->GetLink(linkName);
    GZ_ASSERT(this->link, "Cannot find a link with the specified link_name.");
    this->battery = this->link->Battery(batteryName);
    GZ_ASSERT(this->link, "Cannot find a battery in the link with the specified battery_name. Make sure the batter_name specified in the plugin can be found in the specified link.");

    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);

    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
    this->motor_power_pub = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/consumer/motor_power", 1);
    ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/" + this->model->GetName() + "/joint_states",
        1,
        boost::bind(&MotorConsumerPlugin::OnJointStateMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->joint_state_sub = this->rosNode->subscribe(so);
    this->rosQueueThread = std::thread(std::bind(&MotorConsumerPlugin::QueueThread, this));

    gzlog << "motor consumer loaded\n";
}

void MotorConsumerPlugin::Init()
{
    gzlog << "motor_consumer is initialized\n";
}

void MotorConsumerPlugin::Reset()
{
    gzlog << "motor_consumer is reset\n";
}

double MotorConsumerPlugin::CalculatePower(const sensor_msgs::JointState::ConstPtr &_msg)
{
    int n = sizeof(_msg->velocity)/sizeof(_msg->velocity[0])+1;
    double wheelVel = 0;
    for (int i =0; i<n; i++) {
        wheelVel += std::fabs(_msg->velocity[i]);
    }
    #ifdef MOTOR_CONSUMER_DEBUG
        gzdbg << "motor_consumer:: "
        << n
        << " joints found. Joint velocity:"
        << wheelVel
        << "\n";
    #endif
    return std::max(wheelVel*this->powerLoadRate, this->consumerIdlePower);
}

void MotorConsumerPlugin::OnJointStateMsg(const sensor_msgs::JointState::ConstPtr &_msg)
{
  double motor_power = CalculatePower(_msg);
  this->battery->SetPowerLoad(this->consumerId, motor_power);
  std_msgs::Float64 motor_power_msg;
  motor_power_msg.data = motor_power;
  lock.lock();
  this->motor_power_pub.publish(motor_power_msg);
  lock.unlock();
}

void MotorConsumerPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}