/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

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
