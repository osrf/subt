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

#include <gtest/gtest.h>


#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include <ignition/msgs/boolean.pb.h>


#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>

const std::string worldSdf = R"(
<sdf version="1.6">
  <world name="default">
    <plugin
      filename="libignition-gazebo-user-commands-system.so"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin filename="libGasEmitterDetectorPlugin.so" name="subt::GasEmitter">
      <emitter>
        <type>methane</type>
        <pose>10 0 0 0 0 0</pose>
        <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
      </emitter>
      <emitter>
        <type>oxygen</type>
        <pose>20 0 0 0 0 0</pose>
        <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
      </emitter>
      <emitter>
        <type>propane</type>
        <pose>30 0 0 0 0 0</pose>
        <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
      </emitter>
    </plugin>

    <model name="gas_detector_model">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>1</ixx>
            <iyy>1</iyy>
            <izz>1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name="methane_detector" type="contact">
          <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
            <topic>methane_detector</topic>
            <update_rate>10</update_rate>
            <type>methane</type>
          </plugin>
        </sensor>
        <sensor name="oxygen_detector" type="contact">
          <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
            <topic>oxygen_detector</topic>
            <update_rate>10</update_rate>
            <type>oxygen</type>
          </plugin>
        </sensor>
        <sensor name="propane_detector" type="contact">
          <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
            <topic>propane_detector</topic>
            <update_rate>10</update_rate>
            <type>propane</type>
          </plugin>
        </sensor>
        <sensor name="any_detector" type="contact">
          <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
            <topic>any_detector</topic>
            <update_rate>10</update_rate>
          </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
)";

static bool methane = false;
static bool oxygen = false;
static bool propane = false;
static bool any = false;

void methane_cb(const ignition::msgs::Boolean &_msg)
{
  methane = _msg.data();
}

void oxygen_cb(const ignition::msgs::Boolean &_msg)
{
  oxygen = _msg.data();
}

void propane_cb(const ignition::msgs::Boolean &_msg)
{
  propane = _msg.data();
}

void any_cb(const ignition::msgs::Boolean &_msg)
{
  any = _msg.data();
}

TEST(OxygenGasPlugin, OxygenGasPlugin)
{
  ignition::common::Console::SetVerbosity(4);

  ignition::transport::Node node;
  node.Subscribe("/methane_detector", methane_cb);
  node.Subscribe("/oxygen_detector", oxygen_cb);
  node.Subscribe("/propane_detector", propane_cb);
  node.Subscribe("/any_detector", any_cb);

  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfString(worldSdf);
  auto server = std::make_unique<ignition::gazebo::Server>(serverConfig);
  server->Run(true, 1, false);

  EXPECT_FALSE(methane);
  EXPECT_FALSE(oxygen);
  EXPECT_FALSE(propane);
  EXPECT_FALSE(any);

  {
    ignition::msgs::Pose req;
    req.set_name("gas_detector_model");
    req.mutable_position()->set_x(10);

    ignition::msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/default/set_pose"};

    ignition::transport::Node node;
    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 1, false);
  }

  EXPECT_TRUE(methane);
  EXPECT_FALSE(oxygen);
  EXPECT_FALSE(propane);
  EXPECT_TRUE(any);

  {
    ignition::msgs::Pose req;
    req.set_name("gas_detector_model");
    req.mutable_position()->set_x(20);

    ignition::msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/default/set_pose"};

    ignition::transport::Node node;
    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 1, false);
  }

  EXPECT_FALSE(methane);
  EXPECT_TRUE(oxygen);
  EXPECT_FALSE(propane);
  EXPECT_TRUE(any);

  {
    ignition::msgs::Pose req;
    req.set_name("gas_detector_model");
    req.mutable_position()->set_x(30);

    ignition::msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/default/set_pose"};

    ignition::transport::Node node;
    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 1, false);
  }

  EXPECT_FALSE(methane);
  EXPECT_FALSE(oxygen);
  EXPECT_TRUE(propane);
  EXPECT_TRUE(any);

  {
    ignition::msgs::Pose req;
    req.set_name("gas_detector_model");
    req.mutable_position()->set_x(40);

    ignition::msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/default/set_pose"};

    ignition::transport::Node node;
    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 1, false);
  }

  EXPECT_FALSE(methane);
  EXPECT_FALSE(oxygen);
  EXPECT_FALSE(propane);
  EXPECT_FALSE(any);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

