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

#include <queue>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include "subt_gazebo/FlashLightPlugin.hh"
#include "subt_gazebo/protobuf/lightcommand.pb.h"

namespace gazebo
{
  /// \brief An example usage of FlashLightPlugin class
  // This sample plugin just waits for 10 seconds,
  // then it turns on all lights.
  class LightControlPlugin : public FlashLightPlugin
  {
    /// \brief Called when the plugin is loaded.
    //  It sets the time to wait.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    // Called by ROS service client
    public: bool Control(
      std_srvs::SetBool::Request &req,
      std_srvs::SetBool::Response &res);


    private:
      ros::ServiceServer service;
  };
}
