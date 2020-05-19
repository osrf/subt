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
#include "SdfParser.hh"

using namespace ignition;
using namespace subt;

/// \brief Data structure containing properties of a level
struct Level
{
  /// \brief Position X
  double x;

  /// \brief Position Y
  double y;

  /// \brief Position Z
  double z;

  /// \brief Size x
  double sx;

  /// \brief Size y
  double sy;

  /// \brief Size z
  double sz;

  /// \brief Buffer of a level
  double buffer;

  /// \brief A list of models referenced by this level
  std::vector<std::string> refs;
};

/// \brief A list of models to ignore when generating levels
const std::set<std::string> modelsToIgnore =
{
  "staging_area", "base_station", "artifact_origin"
};

/// \brief Set to true to print debug visuals
bool debugLevels = false;

/// \brief Print debug visuals the levels sdf string
/// \param[in] _levels List of levels to debug
void printDebugVisuals(const std::map<double, Level> &_levels)
{
  std::stringstream out;
  static int debugCounter = 0;

  for (const auto &lIt : _levels)
  {
    auto l = lIt.second;
    out << "    <model name=\"level_debug_" << std::to_string(debugCounter++)
        << "\">\n";
    out << "      <pose>" << l.x << " " << l.y << " " << l.z
                          << " 0 0 0</pose>\n";
    out << "      <link name=\"level_debug_link\">\n";
    out << "        <visual name=\"level_debug_visual\">\n";
    out << "          <transparency>0.2</transparency>\n";
    out << "          <geometry><box><size>\n";
    out << "            " << l.sx << " " << l.sy << " " << l.sz << "\n";
    out << "          </size></box></geometry>\n";
    out << "          <material><diffuse>1 0 0</diffuse></material>\n";
    out << "          <cast_shadows>false</cast_shadows>\n";
    out << "        </visual>\n";
    out << "      </link>\n";
    out << "      <static>true</static>\n";
    out << "    </model>\n";
  }
  std::cout << out.str() << std::endl;
}

/// \brief Get the levels sdf string
/// \param[in] _level input map of levels
std::string levelsStr(const std::map<double, Level> &_levels)
{
  static int levelCounter = 0;
  std::stringstream out;
  for (const auto &lIt : _levels)
  {
    auto l = lIt.second;
    out << "      <level name=\"level" << std::to_string(levelCounter++)
        << "\">\n";

    for (const auto &r : l.refs)
    {
      out << "        <ref>" << r << "</ref>\n";
    }
    out << "        <pose>" << l.x << " " << l.y << " " << l.z
                            << " 0 0 0</pose>\n";
    out << "        <geometry><box><size>" << l.sx << " " << l.sy << " " << l.sz
                            << "</size></box></geometry>\n";
    out << "        <buffer>" << l.buffer << "</buffer>\n";

    out << "      </level>\n";
  }
  return out.str();
}


/// \brief Print the level sdf elements
/// \param[in] _vertexData vector of model info
/// \param[in] _size Tile size, which is used to compute the size of levels
/// At minimum this should be the size of largest tile in the world but larger
/// values can be specified to increase the size of each level.
/// tile in the world.
/// \param[in] _buffer Level buffer size
void printLevels(std::vector<VertexData> &_vertexData,
    const math::Vector3d &_size, double _buffer)
{
  std::map<double, Level> levelX;
  std::map<double, Level> levelY;

  math::Vector3d min(math::MAX_D, math::MAX_D, math::MAX_D);
  math::Vector3d max(-math::MAX_D, -math::MAX_D, -math::MAX_D);

  // get min & max model position values
  for (const auto &vd : _vertexData)
  {
    min.Min(vd.model.Pose().Pos());
    max.Max(vd.model.Pose().Pos());
  }

  // compute level size and position
  math::Vector3d ls = max - min;
  math::Vector3d l = min + ls * 0.5;

  ls += _size * 2.0;

  double levelSizeFactor = 1.875;

  for (const auto &vd : _vertexData)
  {
    std::string name = vd.tileName;

    double x = vd.model.Pose().Pos().X();
    double y = vd.model.Pose().Pos().Y();

    // Create a new level for each unique x
    if (levelX.find(x) == levelX.end())
    {
      double rx = x;
      double ry = l.Y();
      double rz = l.Z();

      double rsx = _size.X() * levelSizeFactor;
      double rsy = ls.Y();
      double rsz = ls.Z();

      Level level;
      level.x = rx;
      level.y = ry;
      level.z = rz;
      level.sx = rsx;
      level.sy = rsy;
      level.sz = rsz;
      level.buffer = _buffer;
      levelX[x] = level;
    }
    levelX[x].refs.push_back(name);

    // Create a new level for each unique y
    if (levelY.find(y) == levelY.end())
    {
      double cx = l.X();
      double cy = y;
      double cz = l.Z();

      double csx = ls.X();
      double csy = _size.Y() * levelSizeFactor;
      double csz = ls.Z();

      Level level;
      level.x = cx;
      level.y = cy;
      level.z = cz;
      level.sx = csx;
      level.sy = csy;
      level.sz = csz;
      level.buffer = _buffer;
      levelY[y] = level;
    }
    levelY[y].refs.push_back(name);
  }

  // print out plugin with all the levels
  std::stringstream out;
  out << "    <!-- Auto generated: level_generator "
      << _size << " " << _buffer << " -->\n";
  out << "    <plugin name=\"ignition::gazebo\" filename=\"dummy\">\n";
  out << levelsStr(levelX);
  out << levelsStr(levelY);
  out << "    </plugin>";
  std::cout << out.str() << std::endl;


  // print debug visuals
  if (debugLevels)
  {
    printDebugVisuals(levelX);
    printDebugVisuals(levelY);
  }
}

/// \brief Main function to generate levels from input sdf file
/// \param[in] _sdfFile Input sdf file.
/// \param[in] _size Tile size, which is used to compute the size of levels
/// At minimum this should be the size of largest tile in the world but larger
/// values can be specified to increase the size of each level.
/// tile in the world.
/// \param[in] _buffer Buffer of level
void generateLevel(const std::string &_sdfFile, const math::Vector3d &_size,
    double _buffer)
{
  std::ifstream file(_sdfFile);
  if (!file.is_open())
  {
    std::cerr << "Failed to read file " << _sdfFile << std::endl;
    return;
  }
  std::string str((std::istreambuf_iterator<char>(file)),
      std::istreambuf_iterator<char>());

  // filter models that are in the staging area
  std::function<bool(const std::string &, const std::string &)>
      filter = [](const std::string &_name,
      const std::string &/*_type*/)
  {
    return modelsToIgnore.count(_name) > 0;
  };

  std::vector<VertexData> vertexData;
  while (!str.empty())
  {
    size_t result = std::string::npos;
    std::string includeStr = SdfParser::Parse("include", str, result);
    if (result == std::string::npos || result > str.size())
      break;

    VertexData vd;
    bool filled = SdfParser::FillVertexData(includeStr, vd, filter);
    if (filled)
      vertexData.push_back(vd);

    str = str.substr(result);
  }

  printLevels(vertexData, _size, _buffer);

  file.close();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 6)
  {
    std::cerr << "Usage: dot_generator <path_to_world_sdf_file> "
              << "<size_x> <size_y> <size_z> <buffer>"
              << std::endl;
    return -1;
  }
  std::string sdfFile = argv[1];

  double sx = std::stod(argv[2]);
  double sy = std::stod(argv[3]);
  double sz = std::stod(argv[4]);
  double buffer = std::stod(argv[5]);

  generateLevel(sdfFile, math::Vector3d(sx, sy, sz), buffer);

  return 0;
}
