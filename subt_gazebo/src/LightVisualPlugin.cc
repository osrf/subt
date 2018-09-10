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

#include <map>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/LightVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the Light class
  class LightVisualPluginPrivate
  {
    /// \brief Map of light names to visualize flag
    public: std::map<std::string, bool> lights;

    /// \brief PreRender connection
    public: event::ConnectionPtr preRenderCon;

    /// \brief PreRender connection
    public: rendering::ScenePtr scene;
  };
}


using namespace gazebo;
using namespace subt;

GZ_REGISTER_VISUAL_PLUGIN(LightVisualPlugin)

/////////////////////////////////////////////////
LightVisualPlugin::LightVisualPlugin()
  : dataPtr(new LightVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
void LightVisualPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent, "LightVisualPlugin parent pointer is NULL");

  this->dataPtr->scene = _parent->GetScene();
  rendering::VisualPtr linkVis = _parent->GetParent();

  if (_sdf->HasElement("light"))
  {
    auto sdfLight = _sdf->GetElement("light");
    while (sdfLight)
    {
      if (sdfLight->HasElement("id") && sdfLight->HasElement("visualize"))
      {
        std::string lightId = sdfLight->Get<std::string>("id");

        size_t pos = 0;
        while ((pos = lightId.find("/", pos)) != std::string::npos)
        {
          lightId = lightId.replace(pos, 1, "::");
          pos += 2;
        }

        bool visualize = sdfLight->Get<bool>("visualize");
        std::string lightName = linkVis->Name() + "::" + lightId;
        this->dataPtr->lights[lightName] = visualize;
      }

      sdfLight = sdfLight->GetNextElement("light");
    }
  }

  if (!this->dataPtr->lights.empty())
  {
    this->dataPtr->preRenderCon =
        event::Events::ConnectPreRender(
        std::bind(&LightVisualPlugin::PreRender, this));
  }
}

/////////////////////////////////////////////////
void LightVisualPlugin::PreRender()
{
  for (auto lightIt = this->dataPtr->lights.begin();
      lightIt != this->dataPtr->lights.end();)
  {
    rendering::LightPtr light =
        this->dataPtr->scene->LightByName(lightIt->first);
    if (light)
    {
      light->ShowVisual(lightIt->second);
      this->dataPtr->lights.erase(lightIt++);
    }
    else
    {
      ++lightIt;
    }
  }

  if (this->dataPtr->lights.empty())
  {
    this->dataPtr->preRenderCon.reset();
    return;
  }
}
