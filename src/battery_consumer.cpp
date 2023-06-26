#include "battery_consumer.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() : consumerId(-1)
{
}

BatteryConsumerPlugin::~BatteryConsumerPlugin()
{
    if (this->battery && this->consumerId !=-1)
        this->battery->RemoveConsumer(this->consumerId);
}

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // TODO: checking whether these elements exists
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM_NAMED("battery_consumer", "A ROS node for Gazebo has not been initialized, "
            "unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
            "gazebo_ros package.");
        return;
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = _model->GetLink(linkName);

    // Create battery
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    this->battery = this->link->Battery(batteryName);

    // Add consumer and sets its power load
    this->powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, powerLoad);

    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));

    this->set_power_load = this->rosNode->advertiseService(this->model->GetName() + "/set_power_load", &BatteryConsumerPlugin::SetConsumerPowerLoad, this);

    gzlog << "consumer loaded\n";

}

void BatteryConsumerPlugin::Init()
{
    gzlog << "consumer is initialized\n";
}

void BatteryConsumerPlugin::Reset()
{
    gzlog << "consumer is reset\n";
}

bool BatteryConsumerPlugin::SetConsumerPowerLoad(gazebo_ros_linear_battery::SetLoad::Request &req,
                                                 gazebo_ros_linear_battery::SetLoad::Response &res)
{
    lock.lock();
    double load = this->powerLoad;
    this->powerLoad = req.power_load;
    this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

    // gzlog << "Power load of consumer has changed from:" << load << ", to:" << this->powerLoad << "\n";

    lock.unlock();
    res.result = true;
    return true;
}