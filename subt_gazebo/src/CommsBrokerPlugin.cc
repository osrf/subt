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
#include <functional>
#include <mutex>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/common/Console.hh>

#include "subt_gazebo/CommsBrokerPlugin.hh"
#include "subt_gazebo/VisibilityTable.hh"
#include "test/test_config.h"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsBrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
  this->commsModel.reset(new subt::CommsModel(
    this->broker.Team(), this->world, _sdf));
  this->maxDataRatePerCycle = this->commsModel->MaxDataRate() *
      this->world->Physics()->GetMaxStepSize();
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CommsBrokerPlugin::OnUpdate, this));

  ignmsg << "Starting SubT comms broker" << std::endl;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  uint32_t maxDataRate = this->maxDataRatePerCycle;
  auto now = gazebo::physics::get_world()->SimTime();
  auto dt = (now - this->lastROSParameterCheckTime).Double();

  // It's time to query the ROS parameter server. We do it every second.
  if (dt >= 2.0 )
  {
    bool value = false;
    ros::param::get("/subt/comms/simple_mode", value);

    if (!this->commsModel->SimpleMode() && value)
    {
      igndbg << "Enabling simple mode comms" << std::endl;
      maxDataRate = UINT32_MAX;
    }
    else if (this->commsModel->SimpleMode() && !value)
    {
      igndbg << "Disabling simple mode comms" << std::endl;
    }

    this->commsModel->SetSimpleMode(value);
    this->lastROSParameterCheckTime = now;

    // // caguero testing
    this->UpdateVisibilityVisual();
  }

  // We need to lock the broker mutex from the outside because "commsModel"
  // accesses its "team" member variable.
  std::lock_guard<std::mutex> lock(this->broker.Mutex());

  // Update the state of the communication model.
  this->commsModel->Update();

  // Send a message to each team member with its updated neighbors list.
  this->broker.NotifyNeighbors();

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  this->broker.DispatchMessages(maxDataRate, this->commsModel->UdpOverhead());
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::Reset()
{
  this->broker.Reset();
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::UpdateVisibilityVisual()
{
  ignition::transport::Node node;
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::BOX);
  markerMsg.mutable_lifetime()->set_sec(2.0);
  markerMsg.mutable_lifetime()->set_nsec(0.0);

  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Green");
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.5, 0.5, 0.5));

  std::string filePath = ignition::common::joinPaths(
    SUBT_GAZEBO_PROJECT_SOURCE_PATH, "worlds", "tunnel_mini.dot");

  subt::VisibilityTable visibilityTable(filePath);
  // auto cost = visibilityTable.Cost(
  //   ignition::math::Vector3d(20, 0, -2.5), ignition::math::Vector3d(80, 0, -17.5));
  // std::cout << "Cost: " << cost << std::endl;
 
  ignition::math::Vector3d from(20, 0, -5);
  auto model = gazebo::physics::get_world()->ModelByName("X1");
  if (model)
  {
    std::cout << "X1 model found" << std::endl;
    from = model->WorldPose().Pos();
    std::cout << "Pos: " << from << std::endl;
    std::cout << "Index: " << visibilityTable.Index(from) << std::endl;
  }
  else
    std::cout << "X1 not found" << std::endl;

  model = gazebo::physics::get_world()->ModelByName("tile_6");
  if (model)
  {
    std::cout << "tile_6 model found" << std::endl;
    from = model->WorldPose().Pos();
    std::cout << "Pos: " << from << std::endl;
    std::cout << "Bounding box: " << model->BoundingBox() << std::endl;
  }

  // uint64_t index = 0;
  // auto cost = visibilityTable.Cost(from, ignition::math::Vector3d(130, 0, 0));
  // if (cost >= 0 && cost <= 20)
  // {
  //   markerMsg.set_id(index++);
  //   ignition::msgs::Set(markerMsg.mutable_pose(),
  //                       ignition::math::Pose3d(130, 0, 0, 0, 0, 0));
  //   node.Request("/marker", markerMsg);
  // }
  // uint64_t index = 0;
  // for (auto z = -20; z <= 0; z += 1)
  //   for (auto y = 0; y <= 0; y += 1)
  //     for (auto x = 10; x <= 140; x += 1)
  //     {
  //       auto cost = visibilityTable.Cost(from,
  //                                        ignition::math::Vector3d(x, y, z));
  //       if (cost >= 0 && cost <= 20)
  //       {
  //         markerMsg.set_id(index++);
  //         ignition::msgs::Set(markerMsg.mutable_pose(),
  //                             ignition::math::Pose3d(x, y, z, 0, 0, 0));
  //         node.Request("/marker", markerMsg);
  //       }
  //     }
}
