/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "ConnectionHelper.hh"
#include "SdfParser.hh"

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
std::string SdfParser::Parse(const std::string &_key, const std::string &_str,
     size_t &_endPos)
{
  std::string elemStartStr = "<" + _key + ">";
  std::string elemEndStr = "</" + _key + ">";

  size_t start = _str.find(elemStartStr);
  if (start == std::string::npos)
    return std::string();
  size_t startIdx = start + elemStartStr.size();
  size_t end = _str.find(elemEndStr, startIdx);
  if (end == std::string::npos)
    return std::string();

  _endPos = end + elemEndStr.size();
  std::string result = _str.substr(startIdx, end - startIdx);
  return result;
}

//////////////////////////////////////////////////
std::string SdfParser::Parse(const std::string &_key, const std::string &_str)
{
  size_t endPos;
  return Parse(_key, _str, endPos);
}

//////////////////////////////////////////////////
bool SdfParser::FillVertexData(const std::string &_includeStr, VertexData &_vd,
  std::function<bool(const std::string &, const std::string &)> &_filter)
{
  // parse name
  std::string name = Parse("name", _includeStr);

  // parse pose
  std::string poseStr = Parse("pose", _includeStr);
  math::Pose3d pose;
  std::stringstream ss(poseStr);
  ss >> pose;

  // parse uri and get model type
  std::string uri = Parse("uri", _includeStr);
  std::string fuelStr =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  size_t fuelIdx = uri.find(fuelStr);
  std::string modelType;
  if (fuelIdx == std::string::npos)
  {
    fuelStr = "https://fuel.ignitionrobotics.org/1.0/openrobotics/models/";
    fuelIdx = uri.find(fuelStr);

    if (fuelIdx == std::string::npos)
    {
      std::cerr  << "Invalid Fuel Index for uri[" << uri << "]\n";
      return false;
    }
  }
  modelType = uri.substr(fuelIdx + fuelStr.size());

  // check if model type is recognized
  if (_filter && _filter(name, modelType) &&
      modelType.find("Blocker") == std::string::npos)
  {
    std::cerr  << "Unrecognized model type[" << modelType << "]\n";
    return false;
  }
  sdf::Model modelSdf;
  modelSdf.SetName(name);
  modelSdf.SetRawPose(pose);

  static int tileId = 0;
  // Try getting the tile id from the tile name first.
  try
  {
    int numIndex = name.rfind("_");
    _vd.id = std::stoi(name.substr(numIndex+1));
  }
  catch (...)
  {
    _vd.id = tileId++;
  }
  _vd.tileType = modelType;
  _vd.tileName = name;
  _vd.model = modelSdf;

  return true;
}

