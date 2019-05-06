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

#include <atomic>
#include <chrono>
#include <thread>
#include <ignition/common/Console.hh>
#include "subt_ign/ControllerPlugin.hh"

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
ControllerPlugin::ControllerPlugin()
{
}

//////////////////////////////////////////////////
bool ControllerPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *elem = _elem->FirstChildElement("name");
  if (!elem)
  {
    ignerr << "[ControllerPlugin] Missing [name] parameter" << std::endl;
    return false;
  }

  this->name = elem->GetText();
  this->client.reset(new subt::CommsClientIgn(this->name, true));
  this->client->Bind(&ControllerPlugin::OnMsg, this);

  ignmsg << this->name << " controller loaded" << std::endl;

  std::function<void()> sendFunct = [&]
  {
    std::string dst = "X1";
    if (this->name == "X1")
      dst = "X2";

    std::string data = "hello from " + this->name;
    this->client->SendTo(data, dst);
  };

  this->timer.Start(1000, sendFunct);

  return true;
}

//////////////////////////////////////////////////
void ControllerPlugin::OnMsg(const std::string &_srcAddress,
  const std::string &/*_dstAddress*/, const uint32_t /*_dstPort*/,
  const std::string &_data)
{
  ignmsg << "[" << this->name << "] Message received from [" << _srcAddress
         << "]: [" << _data << "]" << std::endl;
}
