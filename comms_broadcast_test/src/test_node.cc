#include <string>

#include <ros/ros.h>
#include <subt_communication_broker/subt_communication_client.h>

const int port = 5000;
subt::CommsClient *x1_commsClient;

void X2_IncomingMsgCallback(const std::string & srcAddress,
                            const std::string & dstAddress,
                            const uint32_t dstPort,
                            const std::string & data);

void TimerCallback(const ros::TimerEvent &);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "comms_broadcast_test_node");
  ros::NodeHandle nh;

  // CommsClient for robot X1
  // This CommsClient will send msgs to X2
  std::string robot_X1 = "X1";
  x1_commsClient = new subt::CommsClient(robot_X1);

  // CommsClient for robot X2
  // This CommsClient will receive msgs from X1
  std::string robot_X2 = "X2";
  subt::CommsClient x2_commsClient(robot_X2);
  x2_commsClient.Bind(&X2_IncomingMsgCallback,
                      robot_X2,
                      port);

  auto timer = nh.createTimer(ros::Duration(1.0), &TimerCallback, true);

  ros::spin();
}

void X2_IncomingMsgCallback(const std::string & srcAddress,
                            const std::string & dstAddress,
                            const uint32_t dstPort,
                            const std::string & data)
{
  ROS_INFO_STREAM("Received a message from "
                  << srcAddress
                  << " intended for "
                  << dstAddress
                  << " on port "
                  << dstPort);

  // TODO do something with "data" here
}

void TimerCallback(const ros::TimerEvent &)
{
  ROS_INFO("Sending message...\n");
  x1_commsClient->SendTo("hello", subt::communication_broker::kBroadcast, port);
}
