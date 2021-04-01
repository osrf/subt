#include <ros/ros.h>
#include <subt_communication_broker/subt_communication_client.h>

// This is a test node that allows viewing the effect of comms extender
// See the package's readme for details on how to use it

std::string name;
std::string dstName;

std::string payload;

ros::Time lastDataStamp {0, 0};
size_t dataInWindow {0};

void IncomingMsgCallback(
  const std::string& srcAddress, const std::string& dstAddress,
  const uint32_t dstPort, const std::string& data)
{
  const auto now = ros::Time::now();
  if (now.sec != lastDataStamp.sec)
  {
    ROS_INFO_STREAM("[" << name << ":" << dstPort << "]: received from [" << srcAddress << "] " << dataInWindow << " bytes per second");
    dataInWindow = 0;
  }
  lastDataStamp = now;
  dataInWindow += data.length();
}

void Send(const ros::TimerEvent&, subt::CommsClient& commsClient)
{
  commsClient.SendTo(payload, dstName);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "comms_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("robot_name", name, std::string("COMMS_EXTENDER"));
  pnh.param("dst_name", dstName, std::string("TEAMBASE"));

  payload.resize(1500, 'a');

  subt::CommsClient commsClient(name);
  commsClient.Bind(&IncomingMsgCallback, name);

  auto timer = nh.createTimer(ros::Duration(0.005), boost::bind(&Send, _1, boost::ref(commsClient)));

  ros::spin();
}