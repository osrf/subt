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

#include <tinyxml2.h>

#include <functional>
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <subt_ign/CommsBrokerPlugin.hh>

using namespace ignition;
using namespace subt;
using namespace subt::communication_broker;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::rf_interface::visibilityModel;

/////////////////////////////////////////////////
bool CommsBrokerPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  // Build RF Function
  struct range_model::rf_configuration rangeConfig;
  struct visibilityModel::RfConfiguration visibilityConfig;
  // Build radio configuration (which includes RF pathloss function)
  struct radio_configuration radio;

  const tinyxml2::XMLElement *commsModelElem =
    _elem->FirstChildElement("comms_model");
  if (commsModelElem)
  {
    const tinyxml2::XMLElement *rfConfigElem =
      commsModelElem->FirstChildElement("visibility_config");

    if (rfConfigElem)
    {
      const tinyxml2::XMLElement *elem =
        rfConfigElem->FirstChildElement("visibility_cost_to_fading_exponent");

      if (elem)
      {
        visibilityConfig.visibilityCostToFadingExponent =
          std::stod(elem->GetText());
      }

      elem = rfConfigElem->FirstChildElement("comms_cost_max");
      if (elem)
        visibilityConfig.commsCostMax = std::stod(elem->GetText());

      igndbg << "Loading visibility_config from SDF: \n" << visibilityConfig
        << std::endl;
    }

    rfConfigElem = commsModelElem->FirstChildElement("range_config");
    if (rfConfigElem)
    {
      const tinyxml2::XMLElement *elem =
        rfConfigElem->FirstChildElement("max_range");
      if (elem)
        rangeConfig.max_range = std::stod(elem->GetText());

      elem = rfConfigElem->FirstChildElement("fading_exponent");
      if (elem)
        rangeConfig.fading_exponent = std::stod(elem->GetText());

      elem = rfConfigElem->FirstChildElement("L0");
      if (elem)
        rangeConfig.L0 = std::stod(elem->GetText());

      elem = rfConfigElem->FirstChildElement("sigma");
      if (elem)
        rangeConfig.sigma = std::stod(elem->GetText());

      elem = rfConfigElem->FirstChildElement("scaling_factor");
      if (elem)
        rangeConfig.scaling_factor = std::stod(elem->GetText());

      elem = rfConfigElem->FirstChildElement("range_per_hop");
      if (elem)
        rangeConfig.range_per_hop = std::stod(elem->GetText());

      igndbg << "Loading range_config from SDF: \n" << rangeConfig << std::endl;
    }

    const tinyxml2::XMLElement *radioConfigElem =
      commsModelElem->FirstChildElement("radio_config");

    if (radioConfigElem)
    {
      const tinyxml2::XMLElement *elem =
        radioConfigElem->FirstChildElement("capacity");
      if (elem)
        radio.capacity = std::stod(elem->GetText());

      elem = radioConfigElem->FirstChildElement("tx_power");
      if (elem)
        radio.default_tx_power = std::stod(elem->GetText());

      elem = radioConfigElem->FirstChildElement("modulation");
      if (elem)
        radio.modulation = elem->GetText();

      elem = radioConfigElem->FirstChildElement("noise_floor");
      if (elem)
        radio.noise_floor = std::stod(elem->GetText());

      igndbg << "Loading radio_config from SDF: \n" << radio << std::endl;
    }
  }

  std::string worldName = "default";
  // bool generateTable = false;

  const tinyxml2::XMLElement *elem = _elem->FirstChildElement("world_name");
  if (elem)
  {
    worldName = elem->GetText();
  }
  else
  {
    ignerr << "Missing <world_name>, the GameLogicPlugin will assume a "
      << " world name of 'default'. This could lead to incorrect scoring\n";
  }

  // elem = _elem->FirstChildElement("generate_table");
  // if (elem)
  // {
  //   std::string boolStr = elem->GetText();
  //   generateTable = common::lowercase(boolStr) == "true" || boolStr == "1";
  // }

  // if (generateTable)
  // {
  //   if (!worldName.empty() && !worldDir.empty())
  //   {
  //     subt::VisibilityTable table;
  //     table.Load(worldName, worldDir);
  //     table.Generate();
  //   }
  //   else
  //   {
  //     ignerr << "Unable to generate visibility table because world_name or "
  //       << "world_dir elements are missing.\n";
  //   }
  // }

  // Build RF propagation function options
  std::map<std::string, pathloss_function> pathlossFunctions;

  pathlossFunctions["log_normal_range"] =
      std::bind(&log_normal_received_power,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                rangeConfig);

  // TODO: Maybe only try to instantiate if visibility type is selected
  this->visibilityModel = std::make_unique<VisibilityModel>(
    visibilityConfig, rangeConfig, worldName);

  // Only consider the visibility range if all files (.dot and .dat) are found.
  if (this->visibilityModel->Initialized())
  {
    pathlossFunctions["visibility_range"] =
      std::bind(&VisibilityModel::ComputeReceivedPower,
                this->visibilityModel.get(),
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3);
  }

  // Default comms model type is log_normal_range (will always work)
  std::string commsModelType = "log_normal_range";

  commsModelElem = _elem->FirstChildElement("comms_model");
  if (commsModelElem)
  {
    const tinyxml2::XMLElement *elem =
      commsModelElem->FirstChildElement("comms_model_type");
    if (elem)
    {
      std::string commsModelTypeTmp = elem->GetText();

      // Only allow the specified commsModelType if it is in our map
      // of available functions
      if (pathlossFunctions.find(commsModelTypeTmp) == pathlossFunctions.end())
      {
        ignwarn << "comms_model_type: [" << commsModelTypeTmp
                << "] is not available" << std::endl;
      }
      else
      {
        commsModelType = commsModelTypeTmp;
      }
    }
    else
    {
      ignwarn << "comms_model_type not specified" << std::endl;
    }
  }

  igndbg << "Using [" << commsModelType << "] comms model" << std::endl;

  radio.pathloss_f = pathlossFunctions[commsModelType];
  broker.SetDefaultRadioConfiguration(radio);

  // Set communication function (i.e., the attempt_send function) to
  // use for the broker
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);

  // Build function to get pose from gazebo
  auto updatePoseFunc = [&](const std::string &_name)
  {
    std::map<std::string, ignition::math::Pose3d>::iterator iter =
      this->poses.find(_name);
    if (iter == this->poses.end())
    {
      return std::make_tuple(false,
                             ignition::math::Pose3<double>(),
                             0.0);
    }

    return std::make_tuple(true,
                           iter->second,
                           simTime.Double());
  };
  broker.SetPoseUpdateFunction(updatePoseFunc);
  broker.Start();

  // Subscribe to pose messages.
  this->node.Subscribe("/world/" + worldName + "/pose/info",
      &CommsBrokerPlugin::OnPose, this);

  ignmsg << "Starting SubT comms broker" << std::endl;

  return true;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnPose(const ignition::msgs::Pose_V &_msg)
{
  // Store sim time if present
  if (_msg.has_header() && _msg.header().has_stamp())
  {
    this->simTime.Set(_msg.header().stamp().sec(),
        _msg.header().stamp().nsec());
  }

  // Update all the published poses
  for (int i = 0; i < _msg.pose_size(); ++i)
    this->poses[_msg.pose(i).name()] = ignition::msgs::Convert(_msg.pose(i));

  // Update the visibility graph if needed.
  this->UpdateIfNewBreadcrumbs();

  // double dt = (this->simTime - this->lastROSParameterCheckTime).Double();

  // Todo: Remove this line and enable the block below when ROS parameter
  // server is working.
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);

  // // It's time to query the ROS parameter server. We do it every second.
  // if (dt >= 1.0 )
  // {
  //   bool value = false;
  //   ros::param::get("/subt/comms/simple_mode", value);

  //   if (value)
  //   {
  //     igndbg << "Enabling simple mode comms" << std::endl;

  //     auto f = [](const radio_configuration&,
  //                 const rf_interface::radio_state&,
  //                 const rf_interface::radio_state&,
  //                 const uint64_t&
  //                 ) { return std::make_tuple(true, 0); };

  //     broker.SetCommunicationFunction(f);
  //   }
  //   else
  //   {
  //     igndbg << "Disabling simple mode comms" << std::endl;
  //     broker.SetCommunicationFunction(
  //         &subt::communication_model::attempt_send);
  //   }

  //   this->lastROSParameterCheckTime = this->simTime;
  // }

  // Send a message to each team member with its updated neighbors list.
  this->broker.NotifyNeighbors();

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  this->broker.DispatchMessages();
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::UpdateIfNewBreadcrumbs()
{
  bool newBreadcrumbFound = false;
  for (const auto& [name, pose] : this->poses)
  {
    // New breadcrumb found.
    // A static model is spawned when the breadcrumb is made static
    if (name.find("__breadcrumb__") != std::string::npos &&
        name.find("__static__") != std::string::npos &&
        this->breadcrumbs.find(name) == this->breadcrumbs.end())
    {
      this->breadcrumbs[name] = pose;
      newBreadcrumbFound = true;
    }
  }

  // Update the comms.
  if (newBreadcrumbFound)
  {
    std::set<ignition::math::Vector3d> breadcrumbPoses;
    for (const auto& it : this->breadcrumbs)
      breadcrumbPoses.insert(it.second.Pos());
    this->visibilityModel->PopulateVisibilityInfo(breadcrumbPoses);
    ignmsg << "New breadcrumb detected, visibility graph updated" << std::endl;
  }
}
