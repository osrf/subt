#include <memory>

#include <ignition/msgs/int32_v.pb.h>
#include <ignition/transport/Node.hh>

#include <ctu_cras_norlab_lily_sensor_config_1/LogicalContact.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace subt::lily
{

class LogicalContactBridge : public nodelet::Nodelet
{
  private: void onInit() override
  {
    if (!this->getPrivateNodeHandle().getParam("ign_topic", this->ignTopic))
    {
      ROS_ERROR("LogicalContactBridge: Please, provide parameter ~ign_topic");
      return;
    }

    this->pub = this->getNodeHandle().advertise<ctu_cras_norlab_lily_sensor_config_1::LogicalContact>("logical_contacts", 100);

    ROS_INFO("Subscribing to Ignition topic %s", this->ignTopic.c_str());
    this->node.Subscribe(this->ignTopic, &LogicalContactBridge::OnContacts, this);
  }

  void OnContacts(const ignition::msgs::Int32_V& ignMsg)
  {
    ROS_INFO_ONCE("Received first message on topic %s", this->ignTopic.c_str());

    ctu_cras_norlab_lily_sensor_config_1::LogicalContactPtr msg(new ctu_cras_norlab_lily_sensor_config_1::LogicalContact);
    msg->header.stamp.sec = ignMsg.header().stamp().sec();
    msg->header.stamp.nsec = ignMsg.header().stamp().nsec();
    msg->contacts.resize(ignMsg.data_size());

    size_t i = 0;
    for (const auto& contact : ignMsg.data())
    {
      msg->contacts[i] = contact > 0;
      i += 1;
    }
    this->pub.publish(msg);
  }

  protected: ros::Publisher pub;
  protected: ignition::transport::Node node;
  protected: std::string ignTopic;
};

}

PLUGINLIB_EXPORT_CLASS(subt::lily::LogicalContactBridge, nodelet::Nodelet)