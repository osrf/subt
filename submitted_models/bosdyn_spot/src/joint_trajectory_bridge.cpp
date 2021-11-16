#include <memory>

#include <ignition/msgs/joint_trajectory.pb.h>
#include <ignition/transport/Node.hh>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nodelet/nodelet.h>
#include <ros_ign_bridge/convert.hpp>

#include <pluginlib/class_list_macros.h>

using namespace ros_ign_bridge;


namespace subt
{
/// \brief ROS-Ign bridge for JointTrajectory messages.
class JointTrajectoryBridge : public nodelet::Nodelet
{

  protected: void onInit() override
  {
    if (!this->getPrivateNodeHandle().getParam("ign_topic", this->ignTopic))
    {
      ROS_ERROR("Please, provide parameter ~ign_topic");
      exit(1);
    }

    this->sub = this->getPrivateNodeHandle().subscribe("trajectory", 100, &JointTrajectoryBridge::OnTrajectory, this);

    ROS_INFO("Publishing to Ignition topic %s", this->ignTopic.c_str());
    this->pub = this->node.Advertise<ignition::msgs::JointTrajectory>(this->ignTopic);
  }

  protected: void OnTrajectory(const trajectory_msgs::JointTrajectory& msg)
  {
    ROS_INFO_ONCE("Publishing first message to topic %s", this->ignTopic.c_str());

    ignition::msgs::JointTrajectory ignMsg;

    convert_ros_to_ign(msg.header, (*ignMsg.mutable_header()));

    ignMsg.mutable_points()->Reserve(msg.points.size());
    for (const auto & name : msg.joint_names)
      ignMsg.add_joint_names(name);

    ignMsg.mutable_points()->Reserve(msg.points.size());
    for (const auto& point : msg.points)
    {
      auto* ignPoint = ignMsg.add_points();

      ignPoint->mutable_positions()->Reserve(point.positions.size());
      for (const auto & ros_position : point.positions)
        ignPoint->add_positions(ros_position);

      ignPoint->mutable_velocities()->Reserve(point.velocities.size());
      for (const auto & ros_velocity : point.velocities)
        ignPoint->add_velocities(ros_velocity);

      ignPoint->mutable_accelerations()->Reserve(point.accelerations.size());
      for (const auto & ros_acceleration : point.accelerations)
        ignPoint->add_accelerations(ros_acceleration);

      ignPoint->mutable_effort()->Reserve(point.effort.size());
      for (const auto & ros_effort : point.effort)
        ignPoint->add_effort(ros_effort);

      auto* ign_duration = ignPoint->mutable_time_from_start();
      ign_duration->set_sec(point.time_from_start.sec);
      ign_duration->set_nsec(point.time_from_start.nsec);
    }

    this->pub.Publish(ignMsg);
  }

  protected: ignition::transport::Node::Publisher pub;
  protected: ignition::transport::Node node;
  protected: ros::Subscriber sub;
  protected: std::string ignTopic;
};

}

PLUGINLIB_EXPORT_CLASS(subt::JointTrajectoryBridge, nodelet::Nodelet)