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
#include <subt_ign/VisibilityTable.hh>
#include <subt_ign/VisibilityPlugin.hh>

using namespace subt;

class subt::VisibilityPluginPrivate
{
  public: void OnUpdate();
};

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Load(const tinyxml2::XMLElement * /*_elem*/)
{
}

/////////////////////////////////////////////
void VisibilityPluginPrivate::OnUpdate()
{
  // Hack: When using ConnectWorldCreated, the bounding box of the models are
  // not set properly. Instead, we use ConnectWorldUpdateBegin just once.
  /* \todo(nkoenig) if (gazebo::physics::get_world()->SimTime() > gazebo::common::Time(1))
  {
    subt::VisibilityTable table;
    table.Generate();
    this->worldUpdateConn.reset();

    // Send SIGINT to terminate Gazebo.
    raise(SIGINT);
  }*/
}
