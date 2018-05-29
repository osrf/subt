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

#include <iostream>
#include <string>

#include "subt_gazebo/LightControlClient.hh"
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/protobuf/lightcommand.pb.h"

using namespace subt;

//////////////////////////////////////////////////
LightControlClient::LightControlClient(
  const std::string &_light_name, const std::string &_link_name):
  light_name{_light_name}, link_name{_link_name}
{
  // Sanity check: Verity that light and link names are not empty.
  if (_light_name.empty() || _link_name.empty())
  {
    std::cerr << "LightControlClient::LightControlClient() error: "
              << " Light/Link names shouldn't be empty" << std::endl;
  }
}

//////////////////////////////////////////////////
bool LightControlClient::TurnOn()
{
  msgs::LightCommand msg;
  msg.set_light_name(this->light_name);
  msg.set_link_name(this->link_name);
  msg.set_f_turn_on(true);

  return this->node.Request(kLightCommTopic, msg);
}

//////////////////////////////////////////////////
bool LightControlClient::TurnOff()
{
  msgs::LightCommand msg;
  msg.set_light_name(this->light_name);
  msg.set_link_name(this->link_name);
  msg.set_f_turn_on(false);

  return this->node.Request(kLightCommTopic, msg);
}

//////////////////////////////////////////////////
bool LightControlClient::ChangeDuration(const double &_duration)
{
  msgs::LightCommand msg;
  msg.set_light_name(this->light_name);
  msg.set_link_name(this->link_name);
  msg.set_duration(_duration);

  return this->node.Request(kLightCommTopic, msg);
}

//////////////////////////////////////////////////
bool LightControlClient::ChangeInterval(const double &_interval)
{
  msgs::LightCommand msg;
  msg.set_light_name(this->light_name);
  msg.set_link_name(this->link_name);
  msg.set_interval(_interval);

  return this->node.Request(kLightCommTopic, msg);
}
