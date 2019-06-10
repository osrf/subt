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
#include <ignition/common/Console.hh>
#include <subt_ign/VisibilityTable.hh>
#include <subt_ign/VisibilityPlugin.hh>

using namespace subt;

class subt::VisibilityPluginPrivate
{
};

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  std::cout << "HERE\n";
  subt::VisibilityTable table;
  std::string worldName;
  std::string worldDir;

  const tinyxml2::XMLElement *elem = _elem->FirstChildElement("world_name");

  if (elem)
    worldName = elem->GetText();
  else
    ignerr << "VisibilityPlugin is missing the <world_name> element.\n";

  elem = _elem->FirstChildElement("world_dir");
  if (elem)
    worldDir = elem->GetText();
  else
    ignerr << "VisibilityPlugin is missing the <world_dir> element.\n";

  if (!worldName.empty() && !worldDir.empty())
  {
    std::cout << "WORLD Name[" << worldName << "] World Dir[" << worldDir
              << "]\n";
    table.Load(worldName, worldDir);
    table.Generate();
  }
}
