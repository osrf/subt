/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <subt_msgs/Artifact2.h>

#include "subt_gazebo/BaseStationPlugin.hh"
#include "subt_gazebo/CommsClient.hh"
#include "subt_gazebo/protobuf/artifact.pb.h"

using namespace subt;

GZ_REGISTER_MODEL_PLUGIN(BaseStationPlugin)

//////////////////////////////////////////////////
BaseStationPlugin::BaseStationPlugin()
{
  gzmsg << "Base statation plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void BaseStationPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent, "BaseStationPlugin() error: _parent pointer is NULL");
  GZ_ASSERT(_sdf,    "BaseStationPlugin() error: _sdf pointer is NULL");

  // We pass true to flag this client as private. The goal is to restrict the
  // ignition transport communication to only other nodes running within the
  // same process (e.g.: the GameLogicPlugin).
  this->client.reset(new subt::CommsClient(_parent->GetName(), true));
  this->client->Bind(&BaseStationPlugin::OnArtifact, this);
}

//////////////////////////////////////////////////
void BaseStationPlugin::OnArtifact(const std::string &/*_srcAddress*/,
  const std::string &/*_dstAddress*/, const uint32_t /*_dstPort*/,
  const std::string &_data)
{
  gzmsg << "Artifact reported!" << std::endl;

  char *d = strdup(_data.c_str());

  subt_msgs::Artifact2 rosArtifact;
  uint32_t serialSize = ros::serialization::serializationLength(rosArtifact);
  ros::serialization::IStream stream(reinterpret_cast<uint8_t *>(d), 
    serialSize);
  ros::serialization::Serializer<subt_msgs::Artifact2>::read(stream, rosArtifact);

  // Report this artifact to the scoring plugin.
  subt::msgs::Artifact req;
  req.set_type(rosArtifact.type);
  req.mutable_pose()->mutable_position()->set_x(
    rosArtifact.pose.pose.position.x);
  req.mutable_pose()->mutable_position()->set_y(
    rosArtifact.pose.pose.position.y);
  req.mutable_pose()->mutable_position()->set_z(
    rosArtifact.pose.pose.position.z);
  this->node.Request("/subt/artifacts/new", req);

  free(d);
} 
