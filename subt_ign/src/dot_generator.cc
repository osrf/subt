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

#include <ignition/common/Console.hh>
#include "ConnectionHelper.hh"

using namespace ignition;
using namespace subt;

/// \brief Parse contents of an sdf element
/// \param[in] _key SDF element key
/// \param[in] _str String content of the sdf element
/// \param[out] _endPos end position of the sdf element string
/// \return Value for the input key
std::string parse(const std::string _key, const std::string &_str,
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

/// \brief Parse contents of an sdf element
/// \param[in] _key SDF element key
/// \param[in] _str String content of the sdf element
/// \return Value for the input key
std::string parse(const std::string _key, const std::string &_str)
{
  size_t endPos;
  return parse(_key, _str, endPos);
}

/// \brief Print the DOT file
/// \param[in] _vertexData vector of vertex data containing
/// vertex and edge info
void printGraph(std::vector<VertexData> &_vertexData)
{
  std::stringstream out;
  out << "graph {\n";
  out << "/* ==== Vertices ==== */\n";
//  out << "0  [label=\"0::base_station::BaseStation\"]\n";

  for (auto &vd : _vertexData)
  {
    // update staging area name for compatibility with other subt tools that
    // rely on this naming convention
    std::string name = vd.tileName;
    std::string type = vd.tileType;
    if (type == "Cave Starting Area" ||
        type == "Urban Starting Area")
    {
      type = "base_station";
      name = "BaseStation";
    }
    out << vd.id << "  " << "[label=\"" << vd.id << "::" << type
        << "::" << name << "\"];\n";
  }

  out << "/* ==== Edges ==== */\n";

  for (unsigned int i = 0u; i < _vertexData.size() -1; ++i)
  {
    for (unsigned int j = i+1; j < _vertexData.size(); ++j)
    {
      math::Vector3d point;
      if (subt::ConnectionHelper::ComputePoint(
          &_vertexData[i], &_vertexData[j], point))
      {
        int cost = 1;
        auto tp1 =
            subt::ConnectionHelper::connectionTypes[_vertexData[i].tileType];
        auto tp2 =
            subt::ConnectionHelper::connectionTypes[_vertexData[j].tileType];
        if (tp1 == subt::ConnectionHelper::STRAIGHT &&
            tp2 == subt::ConnectionHelper::STRAIGHT)
          cost = 1;
        else if (tp1 == subt::ConnectionHelper::TURN &&
            tp2 == subt::ConnectionHelper::STRAIGHT)
          cost = 3;
        else if (tp1 == subt::ConnectionHelper::STRAIGHT &&
            tp2 == subt::ConnectionHelper::TURN)
          cost = 3;
        else
          cost = 6;

        out << _vertexData[i].id << " -- " << _vertexData[j].id
            << " " << "[label=" << cost << "];\n" ;
      }
    }
  }

  out << "}";
  std::cout << out.str() << std::endl;
}

/// \brief Fill VertexData from string
/// \param[in] _includeStr input <include> string
/// \param[out] _vd Vertex data to be filled
/// \return True if vertex data is successfully filled, false otherwise
bool fillVertexData(const std::string &_includeStr, VertexData &_vd)
{
  // parse name
  std::string name = parse("name", _includeStr);

  // parse pose
  std::string poseStr = parse("pose", _includeStr);
  math::Pose3d pose;
  std::stringstream ss(poseStr);
  ss >> pose;

  // parse uri and get model type
  std::string uri = parse("uri", _includeStr);
  std::string fuelStr =
      "https://fuel.ignitionrobotics.org/1.0/openrobotics/models/";
  size_t fuelIdx = uri.find(fuelStr);
  std::string modelType;
  if (fuelIdx == std::string::npos)
    return false;
  modelType = uri.substr(fuelIdx + fuelStr.size());

  // check if model type is recognized
  if (subt::ConnectionHelper::connectionPoints.count(modelType) <= 0)
    return false;
  sdf::Model modelSdf;
  modelSdf.SetName(name);
  modelSdf.SetPose(pose);

  static int tileId = 0;
  _vd.id = tileId++;
  _vd.tileType = modelType;
  _vd.tileName = name;
  _vd.model = modelSdf;

  return true;
}

/// \brief Main function to generat DOT from input sdf file
/// \param[in] _sdfFile Input sdf file.
void generateDOT(const std::string &_sdfFile)
{
  std::ifstream file(_sdfFile);
  if (!file.is_open())
  {
    std::cerr << "Failed to read file " << _sdfFile << std::endl;
    return;
  }
  std::string str((std::istreambuf_iterator<char>(file)),
      std::istreambuf_iterator<char>());

  std::vector<VertexData> vertexData;
  while (!str.empty())
  {
    size_t result = std::string::npos;
    std::string includeStr = parse("include", str, result);
    if (result == std::string::npos || result > str.size())
      break;

    VertexData vd;
    bool filled = fillVertexData(includeStr, vd);
    if (filled)
      vertexData.push_back(vd);

    str = str.substr(result);
  }

  printGraph(vertexData);

  file.close();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: dot_generator <path_to_world_sdf_file>"
              << std::endl;
    return -1;
  }
  std::string sdfFile = argv[1];
  generateDOT(sdfFile);

  return 0;
}
