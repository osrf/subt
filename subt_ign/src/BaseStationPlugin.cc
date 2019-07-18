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

#include <ignition/common/Console.hh>

#include "subt_ign/BaseStationPlugin.hh"
#include "subt_ign/CommonTypes.hh"
#include "subt_ign/protobuf/artifact.pb.h"

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
BaseStationPlugin::BaseStationPlugin()
{
  ignmsg << "Base station plugin loaded" << std::endl;
}

BaseStationPlugin::~BaseStationPlugin()
{
  {
    std::lock_guard<std::mutex> lk(this->mutex);
    this->running = false;
  }
  this->cv.notify_all();
  this->ackThread.join();
}


//////////////////////////////////////////////////
bool BaseStationPlugin::Load(const tinyxml2::XMLElement *)
{
  this->client.reset(new subt::CommsClient("base_station", true));
  this->client->Bind(&BaseStationPlugin::OnArtifact, this);

  // Spawn a thread to reply outside of the callback.
  this->ackThread = std::thread([this](){this->RunLoop();});

  return true;
}

//////////////////////////////////////////////////
void BaseStationPlugin::OnArtifact(const std::string &_srcAddress,
  const std::string &/*_dstAddress*/, const uint32_t /*_dstPort*/,
  const std::string &_data)
{
  subt::msgs::Artifact artifact;
  if (!artifact.ParseFromString(_data))
  {
    ignerr << "Error parsing artifact" << std::endl;
    return;
  }

  std::unique_lock<std::mutex> lk(this->mutex);
  this->score = std::make_unique<subt::msgs::ArtifactScore>();
  unsigned int timeout = 1000;
  bool result;

  // Report this artifact to the scoring plugin.
  this->node.Request(kNewArtifactSrv, artifact, timeout, *this->score, result);

  // If successfully reported, forward to requester.
  if (result)
  {
    this->resAddress = _srcAddress;
    this->cv.notify_one();
  }
  else
  {
    this->score.release();
    ignerr << "Error scoring artifact" << std::endl;
  }
}

//////////////////////////////////////////////////
void BaseStationPlugin::RunLoop()
{
  while (this->running)
  {
    std::unique_lock<std::mutex> lk(this->mutex);
    // Two possible conditions, we either have a score or shutdown.
    this->cv.wait_for(lk, std::chrono::milliseconds(100));

    if (this->score)
    {
      igndbg << "Sending Score" << std::endl;
      std::string data;
      this->score->SerializeToString(&data);
      this->client->SendTo(data, this->resAddress);
      this->score.release();
    }
  }
  igndbg << "Terminating run loop" << std::endl;
}

