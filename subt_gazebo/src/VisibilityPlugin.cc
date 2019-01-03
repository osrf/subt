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
#include <csignal>
#include "gazebo/gazebo_config.h"
#include "gazebo/physics/physics.hh"
#include "subt_gazebo/VisibilityTable.hh"
#include "subt_gazebo/VisibilityPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(VisibilityPlugin)

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Init()
{
  this->worldUpdateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VisibilityPlugin::OnUpdate, this));
}

/////////////////////////////////////////////
void VisibilityPlugin::OnUpdate()
{
  // Hack: When using ConnectWorldCreated, the bounding box of the models are
  // not set properly. Instead, we use ConnectWorldUpdateBegin just once.
  if (gazebo::physics::get_world()->SimTime() > gazebo::common::Time(1))
  {
    subt::VisibilityTable table;
    table.Generate();
    this->worldUpdateConn.reset();

    // Send SIGINT to terminate Gazebo.
    raise(SIGINT);
  }
}
