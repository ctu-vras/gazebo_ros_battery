#pragma once

// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/Battery.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include <sdf/Element.hh>

#include <ros/node_handle.h>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerBase : public gazebo::ModelPlugin
{
public:
    ~BatteryConsumerBase() override;
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Reset() override;

protected:
    virtual void Publish(double powerLoad, const gazebo::common::Time& time, const ros::Duration& sampleInterval,
                         double variance);
    void Publish(double powerLoad, const gazebo::common::Time& time, double variance);
    void Publish(double powerLoad, const gazebo::common::Time& time, const ros::Duration& sampleInterval);
    void Publish(double powerLoad, const gazebo::common::Time& time);
    void Publish(double powerLoad);

    virtual void SetEnabled(bool enabled);

    void OnGzEnabledMsg(const ConstAnyPtr& msg);

    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;
    sdf::ElementPtr sdf;
    common::BatteryPtr battery;

    std::string robotNamespace;
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::unique_ptr<ros::NodeHandle> consumerNode;
    ros::Publisher powerLoadPub;

    transport::NodePtr gzNode;
    transport::PublisherPtr gzConsumerIdPub;
    transport::PublisherPtr gzPowerLoadPub;
    transport::SubscriberPtr gzEnableSub;

    uint32_t consumerId {std::numeric_limits<uint32_t>::max()};  //!< Consumer identifier
    std::string consumerName;
    bool enabled {true};  //!< Whether this consumer is enabled or not.
};

}
