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
#include <fstream>
#include <sys/stat.h>

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
  this->worldCreatedConn = event::Events::ConnectWorldCreated(
        boost::bind(&VisibilityPlugin::OnWorldCreated, this));
}

/////////////////////////////////////////////
void VisibilityPlugin::OnWorldCreated()
{
  std::string worldName = gazebo::physics::get_world()->Name();
  // subt::VisibilityTable visibilityTable(filePath);
  subt::VisibilityTable table(worldName);
  //table.Generate();
}
