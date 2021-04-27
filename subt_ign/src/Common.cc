/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <subt_ign/Common.hh>
#include <subt_ign/Config.hh>
#include <ignition/common/Util.hh>
#include <ros/package.h>

namespace subt
{

/////////////////////////////////////////////////
bool ArtifactFromInt(const uint32_t &_typeInt, ArtifactType &_type)
{
  if (_typeInt > (kArtifactTypes.size() - 1))
    return false;

  _type = static_cast<ArtifactType>(_typeInt);
  return true;
}

/////////////////////////////////////////////////
bool ArtifactFromString(const std::string &_name,
                        ArtifactType &_type)
{
  auto pos = std::find_if(
    std::begin(kArtifactTypes),
    std::end(kArtifactTypes),
    [&_name](const typename std::pair<ArtifactType, std::string> &_pair)
    {
      return (std::get<1>(_pair) == _name);
    });

  if (pos == std::end(kArtifactTypes))
    return false;

  _type = std::get<0>(*pos);
  return true;
}

/////////////////////////////////////////////////
bool ArtifactFromPartialString(const std::string &_name,
                               ArtifactType &_type)
{
  auto pos = std::find_if(
    std::begin(kArtifactNames),
    std::end(kArtifactNames),
    [&_name](const auto &_pair)
    {
      return _name.find(std::get<0>(_pair)) != std::string::npos;
    });

  if (pos == std::end(kArtifactNames))
    return false;

  _type = std::get<1>(*pos);
  return true;
}

/////////////////////////////////////////////////
bool StringFromArtifact(const ArtifactType &_type,
                        std::string &_strType)
{
  auto pos = std::find_if(
    std::begin(kArtifactTypes),
    std::end(kArtifactTypes),
    [&_type](const typename std::pair<ArtifactType, std::string> &_pair)
    {
      return (std::get<0>(_pair) == _type);
    });

  if (pos == std::end(kArtifactTypes))
    return false;

  _strType = std::get<1>(*pos);
  return true;
}

/////////////////////////////////////////////////
bool FullWorldPath(const std::string &_worldName,
                   std::string &_worldPath)
{
  _worldPath = "";
  if (_worldName.empty())
  {
    return false;
  }

  std::string worldsDirectory = ignition::common::joinPaths(
    ros::package::getPath("subt_ign"), "worlds");

  const std::string tunnelPrefix = "tunnel_circuit_";
  const std::string urbanPrefix = "urban_circuit_";
  const std::string cavePrefix = "cave_circuit_";
  if (0 == _worldName.compare(0, tunnelPrefix.size(), tunnelPrefix))
  {
    std::string suffix = _worldName.substr(tunnelPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "tunnel_circuit", suffix);
    }
  }
  else if (_worldName.find(urbanPrefix) != std::string::npos)
  {
    std::string suffix = _worldName.substr(urbanPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "urban_circuit", suffix);
    }
  }
  else if (_worldName.find(cavePrefix) != std::string::npos)
  {
    std::string suffix = _worldName.substr(cavePrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "cave_circuit", suffix);
    }
  }
  else if (_worldName.find("simple") == std::string::npos &&
           _worldName.find("_qual") == std::string::npos &&
           _worldName.find("_stix") == std::string::npos &&
           _worldName.find("niosh_") == std::string::npos &&
           _worldName.find("satsop_") == std::string::npos &&
           _worldName.find("finals_") == std::string::npos)
  {
    return false;
  }

  _worldPath = ignition::common::joinPaths(worldsDirectory, _worldName);

  return true;
}

}  // namespace subt
