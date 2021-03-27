#include <memory>

#include <ignition/msgs/contacts.pb.h>
#include <ignition/transport/Node.hh>

#include <ros/ros.h>
#include <bosdyn_spot/ContactsStamped.h>
#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>


namespace subt
{

void ignVectorToMsg(const ignition::msgs::Vector3d& ignV, geometry_msgs::Vector3& rosV)
{
  rosV.x = ignV.x();
  rosV.y = ignV.y();
  rosV.z = ignV.z();
}

/// \brief ROS-Ign bridge for relaying bosdyn_spot::ContactsStamped for a leg.
class FootContactBridge : public nodelet::Nodelet
{

  protected: void onInit() override
  {
    if (!this->getPrivateNodeHandle().getParam("ign_topic", this->ignTopic))
    {
      ROS_ERROR("Please, provide parameter ~ign_topic");
      exit(1);
    }

    this->pub = this->getNodeHandle().advertise<bosdyn_spot::ContactsStamped>("contacts", 100);

    ROS_INFO("Subscribing to Ignition topic %s", this->ignTopic.c_str());
    this->node.Subscribe(this->ignTopic, &FootContactBridge::OnContacts, this);
  }

  protected: void OnContacts(const ignition::msgs::Contacts& ignMsg)
  {
    ROS_INFO_ONCE("Received first message on topic %s", this->ignTopic.c_str());

    bosdyn_spot::ContactsStamped msg;
    msg.header.stamp.sec = ignMsg.header().stamp().sec();
    msg.header.stamp.nsec = ignMsg.header().stamp().nsec();

    for (const auto& contact : ignMsg.contact())
    {
      bosdyn_spot::Contact con;
      con.collision1 = contact.collision1().id();
      con.collision2 = contact.collision2().id();
      con.world = contact.world().id();

      for (const auto& pos: contact.position())
      {
        geometry_msgs::Vector3 p;
        ignVectorToMsg(pos, p);
        con.position.push_back(p);
      }

      for (const auto& norm: contact.normal())
      {
        geometry_msgs::Vector3 n;
        ignVectorToMsg(norm, n);
        con.normal.push_back(n);
      }

      for (const auto& wrench: contact.wrench())
      {
        geometry_msgs::Wrench w;
        if (wrench.body_1_wrench().has_force())
          ignVectorToMsg(wrench.body_1_wrench().force(), w.force);
        if (wrench.body_1_wrench().has_torque())
          ignVectorToMsg(wrench.body_1_wrench().torque(), w.torque);
        con.body1_wrench.push_back(w);

        geometry_msgs::Wrench w2;
        if (wrench.body_2_wrench().has_force())
          ignVectorToMsg(wrench.body_2_wrench().force(), w2.force);
        if (wrench.body_2_wrench().has_torque())
          ignVectorToMsg(wrench.body_2_wrench().torque(), w2.torque);
        con.body2_wrench.push_back(w2);
      }
      msg.contacts.push_back(con);
    }
    this->pub.publish(msg);
  }

  protected: ros::Publisher pub;
  protected: ignition::transport::Node node;
  protected: std::string ignTopic;
};

}

PLUGINLIB_EXPORT_CLASS(subt::FootContactBridge, nodelet::Nodelet)