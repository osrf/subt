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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "subt_gazebo/GameLogicPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(GameLogicPlugin)

/////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void GameLogicPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  GZ_ASSERT(_world, "GameLogicPlugin world pointer is NULL");
  this->world = _world;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WindPlugin::OnUpdate, this));

  gzmsg << "Starting SubT..." << std::endl;
}

/////////////////////////////////////////////////
void WindPlugin::OnUpdate()
{
}
