// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

void poseCallback(const tf2_msgs::TFMessage &_msg)
{
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(_msg.transforms);
}

void poseStaticCallback(const tf2_msgs::TFMessage &_msg)
{
   static tf2_ros::StaticTransformBroadcaster brStatic;
   brStatic.sendTransform(_msg.transforms);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "pose_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("pose", 10, &poseCallback);

  ros::Subscriber subStatic = node.subscribe(
      "pose_static", 10, &poseStaticCallback);

  ros::spin();

  return 0;
}
