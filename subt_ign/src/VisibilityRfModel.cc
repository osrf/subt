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

#include <subt_rf_interface/subt_rf_model.h>
#include <ignition/common/Console.hh>
#include <subt_ign/VisibilityRfModel.hh>

using namespace subt;
using namespace rf_interface;
using namespace visibilityModel;

/////////////////////////////////////////////
VisibilityModel::VisibilityModel(
    visibilityModel::RfConfiguration _visibilityConfig,
    range_model::rf_configuration _rangeConfig,
    const std::string &_worldName)
    : visibilityConfig(_visibilityConfig),
      defaultRangeConfig(_rangeConfig)
{
  if (!this->visibilityTable.Load(_worldName))
  {
    ignerr << "Unable to load visibility table data files\n";
    return;
  }

  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::HOST);
  this->node.Advertise("/subt/comms_model/visualize",
      &VisibilityModel::VisualizeVisibility, this, opts);

  // Subscribe to pose messages.
  this->node.Subscribe("/world/" + _worldName + "/pose/info",
      &VisibilityModel::OnPose, this);

  this->initialized = true;
}

/////////////////////////////////////////////
bool VisibilityModel::Initialized() const
{
  return this->initialized;
}

/////////////////////////////////////////////
rf_power VisibilityModel::ComputeReceivedPower(const double &_txPower,
                                               radio_state &_txState,
                                               radio_state &_rxState)
{
  // Use this->visibilityTable.Cost(_txState, _rxState) to compute
  // pathloss and thus, received power
  double visibilityCost = this->visibilityTable.Cost(_txState.pose.Pos(),
      _rxState.pose.Pos());

  range_model::rf_configuration localConfig = this->defaultRangeConfig;

  // Augment fading exponent based on visibility cost
  localConfig.fading_exponent +=
  this->visibilityConfig.visibilityCostToFadingExponent * visibilityCost;

  double range = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  rf_power rx = range_model::log_normal_received_power(_txPower,
                                                       _txState,
                                                       _rxState,
                                                       localConfig);
  igndbg << "Range: " << range << ", Exp: " << localConfig.fading_exponent
    << ", TX: " << _txPower << ", RX: " << rx.mean << std::endl;

  return std::move(rx);
}

/////////////////////////////////////////////
void VisibilityModel::PopulateVisibilityInfo(
  const std::set<ignition::math::Vector3d> &_relayPoses)
{
  this->visibilityTable.PopulateVisibilityInfo(_relayPoses);
}

/////////////////////////////////////////////
bool VisibilityModel::VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                                          ignition::msgs::Boolean &_rep)
{
  _rep.set_data(false);

  std::map<int, ignition::msgs::Marker> perCostMarkers;

  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  markerMsg.mutable_lifetime()->set_sec(0.0);
  markerMsg.mutable_lifetime()->set_nsec(0.0);

  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  // Available colors
  //
  // RedGlow, YellowGlow, GreenGlow, TurquoiseGlow, BlueGlow
  // High (good)                                    Low (bad)
  std::map<int, std::string> indexToColor;
  indexToColor[0] = "Gazebo/RedGlow";
  indexToColor[1] = "Gazebo/YellowGlow";
  indexToColor[2] = "Gazebo/GreenGlow";
  indexToColor[3] = "Gazebo/TurquoiseGlow";
  indexToColor[4] = "Gazebo/BlueGlow";

  for (int i = 0; i < 5; ++i) {
    markerMsg.set_id(i);

    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    matMsg->mutable_script()->set_name(indexToColor[i]);
    ignition::msgs::Set(markerMsg.mutable_scale(),
                        ignition::math::Vector3d(1.0, 1.0, 1.0));

    perCostMarkers.insert(std::make_pair(i, markerMsg));
  }

  std::string modelName = _req.data();
  std::map<std::string, ignition::math::Pose3d>::iterator iter =
    this->poses.find(modelName);
  if (iter == this->poses.end())
  {
    ignerr << "[" << modelName << "] model not found" << std::endl;
    return true;
  }

  ignition::math::Vector3d from = iter->second.Pos();

  for (auto const &entry : this->visibilityTable.Vertices())
  {
    const auto &toTuple = entry.first;
    ignition::math::Vector3d to = ignition::math::Vector3d(
      std::get<0>(toTuple), std::get<1>(toTuple), std::get<2>(toTuple));
    double cost = this->visibilityTable.Cost(from, to);
    if (cost <= this->visibilityConfig.commsCostMax)
    {
      auto m = perCostMarkers.find(static_cast<int>(
          (((this->visibilityConfig.commsCostMax+1.0)/5.0) *
           cost/10.0)) );

      if (m == perCostMarkers.end())
      {
        ignwarn << "Have not pre-allocated a marker for cost: " << cost
          << " ("
          << (((this->visibilityConfig.commsCostMax+1.0)/5.0)*cost/10.0)
          << ")\n";
        continue;
      }

      ignition::msgs::Set(m->second.add_point(),
          ignition::math::Vector3d(to.X(), to.Y(), to.Z()));
    }
  }

  this->node.Request("/marker", perCostMarkers[0]);
  this->node.Request("/marker", perCostMarkers[1]);
  this->node.Request("/marker", perCostMarkers[2]);
  this->node.Request("/marker", perCostMarkers[3]);
  this->node.Request("/marker", perCostMarkers[4]);

  _rep.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void VisibilityModel::OnPose(const ignition::msgs::Pose_V &_msg)
{
  // Check the all the published poses
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    const ignition::msgs::Pose &pose = _msg.pose(i);
    this->poses[pose.name()] = ignition::msgs::Convert(pose);
  }
}
