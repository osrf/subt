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

#include <algorithm>
#include <string>
#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/World.hh>
#include <ignition/common.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include "subt_gazebo/CommsModel.hh"

using namespace subt;

//////////////////////////////////////////////////
CommsModel::CommsModel(TeamMembershipPtr _team,
    gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  : team(_team),
    world(_world)
{
  GZ_ASSERT(_world, "CommsModel() error: _world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsModel() error: _sdf pointer is NULL");

  this->LoadParameters(_sdf);

  if (!this->visibilityTable.Load())
    return;

  // Debug service only available in the Gazebo host.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::HOST);
  this->node.Advertise(kVisualizeCommsModelSrv,
    &CommsModel::VisualizeVisibility, this, opts);
}

//////////////////////////////////////////////////
void CommsModel::Update()
{
  this->UpdateVisibilityPairs();

  this->InitializeVisibility();

  // Decide if each member of the team enters into a comms outage.
  this->UpdateOutages();

  // Update the visibility state between vehicles.
  // Make sure that this happens after UpdateOutages().
  this->UpdateVisibility();

  // Update the neighbors list of each member of the team.
  // Make sure that this happens after UpdateVisibility().
  this->UpdateNeighbors();
}

//////////////////////////////////////////////////
uint32_t CommsModel::MaxDataRate() const
{
  return this->commsDataRateMax;
}

//////////////////////////////////////////////////
uint16_t CommsModel::UdpOverhead() const
{
  return this->udpOverhead;
}

//////////////////////////////////////////////////
bool CommsModel::SimpleMode() const
{
  return this->simpleMode;
}

//////////////////////////////////////////////////
void CommsModel::SetSimpleMode(const bool _value)
{
  this->simpleMode = _value;
}

//////////////////////////////////////////////////
void CommsModel::LoadParameters(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "CommsModel::LoadParameters() error: _sdf pointer is NULL");

  if (_sdf->HasElement("comms_model"))
  {
    auto const &commsModelElem = _sdf->GetElement("comms_model");

    if (commsModelElem->HasElement("neighbor_distance_min"))
    {
      this->neighborDistanceMin =
        commsModelElem->Get<double>("neighbor_distance_min");
    }
    if (commsModelElem->HasElement("neighbor_distance_max"))
    {
      this->neighborDistanceMax =
        commsModelElem->Get<double>("neighbor_distance_max");
    }
    if (commsModelElem->HasElement("comms_distance_min"))
    {
      this->commsDistanceMin =
        commsModelElem->Get<double>("comms_distance_min");
    }
    if (commsModelElem->HasElement("comms_distance_max"))
    {
      this->commsDistanceMax =
        commsModelElem->Get<double>("comms_distance_max");
    }
    if (commsModelElem->HasElement("comms_drop_probability_min"))
    {
      this->commsDropProbabilityMin =
        commsModelElem->Get<double>("comms_drop_probability_min");
    }
    if (commsModelElem->HasElement("comms_drop_probability_max"))
    {
      this->commsDropProbabilityMax =
        commsModelElem->Get<double>("comms_drop_probability_max");
    }
    if (commsModelElem->HasElement("comms_outage_probability"))
    {
      this->commsOutageProbability =
        commsModelElem->Get<double>("comms_outage_probability");
    }
    if (commsModelElem->HasElement("comms_outage_duration_min"))
    {
      this->commsOutageDurationMin =
        commsModelElem->Get<double>("comms_outage_duration_min");
    }
    if (commsModelElem->HasElement("comms_outage_duration_max"))
    {
      this->commsOutageDurationMax =
        commsModelElem->Get<double>("comms_outage_duration_max");
    }
    if (commsModelElem->HasElement("comms_data_rate_max"))
    {
      this->commsDataRateMax =
        commsModelElem->Get<double>("comms_data_rate_max");
    }

    if (commsModelElem->HasElement("comms_cost_max"))
    {
      this->commsCostMax =
        commsModelElem->Get<double>("comms_cost_max");
    }
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateOutages()
{
  gazebo::common::Time curTime = this->world->SimTime();

  // In case we reset simulation.
  if (curTime <= this->lastUpdateTime)
  {
    this->lastUpdateTime = curTime;
    return;
  }

  // Get elapsed time since the last update.
  auto dt = curTime - this->lastUpdateTime;

  for (auto const &robot : (*this->team))
  {
    auto address = robot.first;
    auto teamMember = robot.second;

    // Check if I am currently on a temporary outage.
    if (teamMember->onOutage &&
        teamMember->onOutageUntil != gazebo::common::Time::Zero)
    {
      // Check if the outage should finish.
      if (this->world->SimTime() >= teamMember->onOutageUntil)
      {
        teamMember->onOutage = false;

        // Debug output.
        // gzdbg << "[" << curTime << "] Robot " << address
        //       << " is back from an outage." << std::endl;
      }
    }
    else if (!teamMember->onOutage)
    {
      // Check if we should go into an outage.
      // Notice that "commsOutageProbability" specifies the probability of going
      // into a comms outage at each second. This probability is adjusted using
      // the elapsed time since the last call to "UpdateOutages()".
      if ((ignition::math::equal(this->commsOutageProbability, 1.0)) ||
          (ignition::math::Rand::DblUniform(0.0, 1.0) <
           this->commsOutageProbability * dt.Double()))
      {
        teamMember->onOutage = true;
        // Debug output.
        // gzdbg << "[" << curTime << "] Robot " << address
        //       << " has started an outage." << std::endl;

        // Decide the duration of the outage.
        if (this->commsOutageDurationMin < 0 ||
            this->commsOutageDurationMax < 0)
        {
          // Permanent outage.
          teamMember->onOutageUntil = gazebo::common::Time::Zero;
        }
        else
        {
          // Temporal outage.
          teamMember->onOutageUntil = curTime +
            ignition::math::Rand::DblUniform(
              this->commsOutageDurationMin,
              this->commsOutageDurationMax);
        }
      }
    }
  }

  this->lastUpdateTime = curTime;
}

//////////////////////////////////////////////////
void CommsModel::InitializeVisibility()
{
  // Initialize visibility.
  for (auto const &robotA : (*this->team))
  {
    auto addressA = robotA.second->address;
    for (auto const &robotB : (*this->team))
    {
      auto addressB = robotB.second->address;
      this->visibility[std::make_pair(addressA, addressB)] = false;
    }
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateVisibility()
{
  // All combinations between a pair of vehicles.
  for (auto const &keyA : this->visibilityPairs)
  {
    auto addressA = keyA.first;
    auto addressB = keyA.second;
    auto keyB = std::make_pair(addressB, addressA);

    // Sanity check: The model pointers should exist.
    if (!(*team)[addressA]->model || !(*team)[addressB]->model)
    {
      this->visibility[keyA] = false;
      this->visibility[keyB] = false;
      continue;
    }

    auto poseA = (*team)[addressA]->model->WorldPose();
    auto poseB = (*team)[addressB]->model->WorldPose();

    // Using distance.
    // this->visibility[keyA] =
    //   this->simpleMode ||
    //   poseA.Pos().Distance(poseB.Pos()) <= this->commsDistanceMax;

    // Using graph and cost between the sections.
    this->visibility[keyA] =
      this->simpleMode ||
      this->visibilityTable.Cost(poseA.Pos(), poseB.Pos()) <=
        this->commsCostMax;

    // Update the symmetric case.
    this->visibility[keyB] = this->visibility[keyA];
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateNeighbors()
{
  // Update the list of neighbors for each robot.
  for (const auto &addr : this->addresses)
    this->UpdateNeighborList(addr);
}

//////////////////////////////////////////////////
void CommsModel::UpdateNeighborList(const std::string &_address)
{
  GZ_ASSERT(this->team->find(_address) != this->team->end(),
            "_address not found in the team.");

  auto teamMember = (*this->team)[_address];
  auto myPose = teamMember->model->WorldPose();

  // Initialize the neighbors list.
  teamMember->neighbors.clear();

  // Decide whether this node goes into our neighbor list.
  for (auto const &member : (*this->team))
  {
    // Where is the other node?
    auto other = member.second;

    // Both robots are in an outage.
    if (teamMember->onOutage || other->onOutage)
    {
      continue;
    }

    // Do not include myself in the list of neighbors.
    if (other->address == _address)
      continue;

    // ToDo: Check if there's line of sight between the two vehicles.

    auto key = std::make_pair(_address, other->address);
    GZ_ASSERT(this->visibility.find(key) != this->visibility.end(),
      "vehicle key not found in visibility");

    bool visible = this->visibility[key];
    if (!visible)
    {
      continue;
    }

    if (this->simpleMode)
    {
      teamMember->neighbors[member.first] = 1.0;
      continue;
    }

    auto otherPose = other->model->WorldPose();
    double dist = myPose.Pos().Distance(otherPose.Pos());

    auto neighborDist = dist;
    auto commsDist = dist;

    if ((this->neighborDistanceMin > 0.0) &&
        (this->neighborDistanceMin > neighborDist))
    {
      continue;
    }
    if ((this->neighborDistanceMax >= 0.0) &&
        (this->neighborDistanceMax < neighborDist))
    {
      continue;
    }

    // Now apply the comms model to compute a probability of a packet from
    // this neighbor arriving successfully.
    auto commsProb = 1.0;

    if ((this->commsDistanceMin > 0.0) &&
        (this->commsDistanceMin > commsDist))
    {
      commsProb = 0.0;
    }
    if ((commsProb > 0.0) &&
        (this->commsDistanceMax >= 0.0) &&
        (this->commsDistanceMax < commsDist))
    {
      commsProb = 0.0;
    }

    if (commsProb > 0.0)
    {
      // We made it through the outage, distance, and obstacle filters.
      // Compute a drop probability between these two nodes for this
      // time step.
      commsProb = 1.0 - ignition::math::Rand::DblUniform(
        this->commsDropProbabilityMin,
        this->commsDropProbabilityMax);
    }

    // Stuff the resulting information into our local representation of this
    // node; we'll refer back to it later when processing messages sent
    // between nodes.
    // Also a message containing the neighbor list (not the probabilities)
    // will be sent out below, to allow robot controllers to query the
    // neighbor list.
    teamMember->neighbors[member.first] = commsProb;
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateVisibilityPairs()
{
  this->addresses.clear();
  this->visibilityPairs.clear();

  for (auto const &robotA : (*this->team))
  {
    auto addressA = robotA.second->address;
    this->addresses.push_back(addressA);
    for (auto const &robotB : (*this->team))
    {
      auto addressB = robotB.second->address;
      std::pair<std::string, std::string> aPair(addressA, addressB);
      std::pair<std::string, std::string> aPairInverse(addressB, addressA);

      // Do not include this case.
      if (addressA == addressB)
        continue;

      // Check if we already have the symmetric pair stored.
      if (std::find(this->visibilityPairs.begin(), this->visibilityPairs.end(),
            aPairInverse) != this->visibilityPairs.end())
      {
        continue;
      }

      this->visibilityPairs.push_back(aPair);
    }
  }
}

/////////////////////////////////////////////////
bool CommsModel::VisualizeVisibility(const ignition::msgs::StringMsg &_req,
  ignition::msgs::Boolean &_rep)
{
  _rep.set_data(false);

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

  std::string modelName = _req.data();
  auto model = this->world->ModelByName(modelName);
  if (!model)
  {
    ignerr << "[" << modelName << "] model not found" << std::endl;
    return true;
  }

  ignition::math::Vector3d from = model->WorldPose().Pos();

  uint64_t index = 0;
  for (auto z = VisibilityTable::kMinZ; z <= VisibilityTable::kMaxZ; ++z)
    for (auto y = VisibilityTable::kMinY; y <= VisibilityTable::kMaxY; ++y)
      for (auto x = VisibilityTable::kMinX; x <= VisibilityTable::kMaxX; ++x)
      {
        ignition::math::Vector3d to = ignition::math::Vector3d(x, y, z);
        double cost = this->visibilityTable.Cost(from, to);
        if (cost >= 0 && cost <= 10)
        {
          markerMsg.set_id(index++);
          ignition::msgs::Set(markerMsg.mutable_pose(),
                              ignition::math::Pose3d(x, y, z, 0, 0, 0));
          this->node.Request("/marker", markerMsg);
        }
      }

  _rep.set_data(true);
  return true;
}
