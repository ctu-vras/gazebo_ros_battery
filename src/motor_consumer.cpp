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

#include "ROS_debugging.h"
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
    #ifdef MOTOR_CONSUMER_DEBUG
        gzdbg << "started loading motor consumer \n";
    #endif

    // check if the ros is up!
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
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

    #ifdef MOTOR_CONSUMER_DEBUG
        gzdbg << "motor consumer loaded \n";
    #endif

    ROS_GREEN_STREAM("motor consumer loaded");

}

void MotorConsumerPlugin::Init()
{
#ifdef MOTOR_CONSUMER_DEBUG
    gzdbg << "motor_consumer is initialized \n";
#endif
    ROS_GREEN_STREAM("motor_consumer is initialized");
}

void MotorConsumerPlugin::Reset()
{
#ifdef MOTOR_CONSUMER_DEBUG
    gzdbg << "motor_consumer is reset \n";
#endif
    ROS_GREEN_STREAM("motor_consumer is reset");
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
        << " wheels found. Wheel velocity:"
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