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

#include "subt_gazebo/RosFlashLightPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
void RosFlashLightPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // === must call this ===
  LedPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so'"
      << " in the gazebo_ros package)");
    return;
  }

  gzmsg << "Plugin Loaded: RosFlashLightPlugin" << std::endl;

  // Service name is renamed if an alternative one is given in SDF.
  std::string serviceName = "light_control";
  if (_sdf->HasElement("service_name"))
  {
    serviceName = _sdf->Get<std::string>("service_name");
  }

  if (serviceName[0] != '/')
  {
    std::string prefix = _parent->GetScopedName();
    int pos = prefix.find("::");
    while (pos > 0)
    {
      prefix.replace(pos, 2, "/");
      pos = prefix.find("::");
    }
    serviceName = "/" + prefix + "/" + serviceName;
  }
  gzmsg << "service name: " << serviceName << std::endl;

  // ROS service to receive a command to control the light
  ros::NodeHandle n;
  this->service
    = n.advertiseService(serviceName, &RosFlashLightPlugin::Control, this);
}

//////////////////////////////////////////////////
bool RosFlashLightPlugin::Control(
  std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  if (_req.data)
  {
    _res.success = this->TurnOnAll();
  }
  else
  {
    _res.success = this->TurnOffAll();
  }

  return _res.success;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RosFlashLightPlugin)
