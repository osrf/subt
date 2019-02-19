#include <ros/ros.h>
#include <subt_communication_broker/subt_communication_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subt_comms_test");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string address;
  pnh.param("address", address, std::string("1"));

  std::string dest;
  pnh.param("destination", dest, std::string("2"));

  auto communication_cb = [](const std::string &srcAddress,
                             const std::string &dstAddress,
                             const uint32_t dstPort,
                             const std::string& data) {

    ROS_INFO("Got message from %s on %s:%u: %s",
             srcAddress.c_str(),
             dstAddress.c_str(),
             dstPort,
             data.c_str());
  };

  subt::CommsClient client(address);

  if(!client.Bind(communication_cb)) {
    ROS_ERROR("Failed to bind!");
    ros::shutdown();
  }

  ROS_INFO("Starting to transmit");

  ros::Rate r(1.0);
  while(ros::ok()) {

    ROS_INFO("Sending...");

    client.SendTo("Hello World", dest);

    r.sleep();
  }

}
