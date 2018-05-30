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

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/LightControlPlugin.hh"
#include "subt_gazebo/protobuf/lightcommand.pb.h"

using namespace gazebo;
using namespace subt;

//////////////////////////////////////////////////
void LightControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // === must call this ===
  FlashLightPlugin::Load(_parent, _sdf);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&LightControlPlugin::OnUpdate, this));

  gzmsg << "Starting light controller" << std::endl;

  // Advertise a oneway service for centralizing all message requests.
  if (!node.Advertise(kLightCommTopic, &LightControlPlugin::OnMessage, this))
  {
    gzerr << "Error advertising service [" << kLightCommTopic << "]"
          << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void LightControlPlugin::OnUpdate()
{
  // === must call this ===
  FlashLightPlugin::OnUpdate();

  std::lock_guard<std::mutex> lk(this->mutex);
  this->ProcessIncomingMsgs();
}

/////////////////////////////////////////////////
void LightControlPlugin::ProcessIncomingMsgs()
{
  while (!this->incomingMsgs.empty())
  {
    // Execute the received command.
    auto const &msg = this->incomingMsgs.front();
    std::string light_name = msg.light_name();
    std::string link_name = "";
    if (msg.has_link_name())
    {
      link_name = msg.link_name();
    }

    // Command to turn on/off
    if (msg.has_f_turn_on())
    {
      if (light_name.empty())
      {
        if (msg.f_turn_on())
        {
          this->TurnOnAll();
        }
        else
        {
          this->TurnOffAll();
        }
      }
      else
      {
        if (msg.f_turn_on())
        {
          this->TurnOn(light_name, link_name);
        }
        else
        {
          this->TurnOff(light_name, link_name);
        }
      }
    }
    // Command to change duration/interval
    if (msg.has_duration())
    {
      this->ChangeDuration(light_name, link_name, msg.duration());
    }
    if (msg.has_interval())
    {
      this->ChangeInterval(light_name, link_name, msg.interval());
    }

    this->incomingMsgs.pop();
  }
}

/////////////////////////////////////////////////
void LightControlPlugin::OnMessage(const subt::msgs::LightCommand &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.push(_req);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LightControlPlugin)
