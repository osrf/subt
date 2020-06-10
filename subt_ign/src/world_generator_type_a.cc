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

#include <list>
#include <string>
#include <sstream>
#include <unistd.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/ClientConfig.hh>

#include "ConnectionHelper.hh"

using namespace ignition;
using namespace subt;

namespace subt {

/// \brief A section of a world
class WorldSection
{
  /// \brief Unique id
  public: size_t id;

  /// \brief A list of tiles in the world section
  public: std::vector<VertexData> tiles;

  /// \brief A list of connection points and their orientation from +X
  /// This field is manually filled in CreatePrefabWorldSections().
  public: std::vector<std::pair<math::Vector3d, math::Quaterniond>>
      connectionPoints;

  /// \brief A list of bounding boxes for the tiles in this world section
  /// This field is auto populated by PreprocessTiles().
  public: std::vector<math::AxisAlignedBox> boundingboxes;

  /// \brief Pose of this world section from world origin
  public: math::Pose3d pose;
};

/// \brief Helper class to generate a world from tiles
class WorldGenerator
{
  /// \brief Set the seed for generating random numbers
  /// \param[in] _seed Seed to set
  public: void SetSeed(int _seed);

  /// \brief Set the output sdf file to be saved
  /// \param[in] _file Path to save the file
  public: void SetOutputFile(const std::string &_file);

  /// \brief Set the min number of tiles to include in the generated world
  /// \param-in] _tileCount Min number of tiles
  public: void SetMinTileCount(int _tileCount);

  /// \brief Set whether to enable GUI in the generate world sdf
  /// \param[in] _gui True to enable gui
  public: void SetEnableGUI(bool _gui);

  /// \brief Set the world name
  /// \param[in] _worldName World name
  public: void SetWorldName(const std::string &_worldName);

  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Create a collection of prefab world sections which are used
  /// to create the final generated world
  private: void CreatePrefabWorldSections();

  /// \brief Helper function to create a world section from an individual file
  /// \param[in] _type Type of tile
  /// \param[in] _entry Entry position of this tile. This should be one of the
  /// connection points of the tile
  /// \param[in] _rot Rotation to be applied to tile so that its entry point is
  /// in +X direction
  private: WorldSection CreateWorldSectionFromTile(const std::string &_type,
      const math::Vector3d &_entry, const math::Quaterniond &_rot);

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  private: void PreprocessTiles();

  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  private: WorldSection SelectWorldSection();

  /// \brief Intersection check between a section to be added to the world
  /// and all the sections that have already been added.
  /// \param[in] _section Section to add to the world.
  /// \param[in] _pose Pose to add the section
  /// \param[in] _addedSections Sections already in the world
  private: bool IntersectionCheck(WorldSection &_section,
      const math::Pose3d _pose,
      const std::vector<WorldSection> &_addedSections);

  /// \brief Get the top part of the world sdf string
  private: std::string WorldTopStr() const;

  /// \brief Get GUI plugin string
  private: std::string WorldGUIStr() const;

  /// \brief Get the bottom part of the world sdf string
  private: std::string WorldBottomStr() const;

  /// \brief A list of connection types used by this world generator
  public: std::map<std::string, subt::ConnectionHelper::ConnectionType>
      tileConnectionTypes;

  /// \brief A list of tile bounding box data for each tile connection type
  public: std::map<std::string, math::AxisAlignedBox>
      tileBoundingBoxes;

  /// \brief A collection of prefab world sections
  public: std::vector<WorldSection> worldSections;

  /// \brief Seed
  private: int seed = 0;

  /// \brief File to save the world sdf to
  private: std::string outputFile = "out.sdf";

  /// \brief Min number of tiles in include in the world
  private: int minTileCount = 10;

  /// \brief Flag to enable gui
  private: bool gui = false;

  /// \brief Name of generated world
  private: std::string worldName = "cave_circuit";
};
}

//////////////////////////////////////////////////
math::AxisAlignedBox transformAxisAlignedBox(
    const math::AxisAlignedBox &_box, const math::Pose3d &_pose)
{
  if (_box == math::AxisAlignedBox())
    return _box;

  // transform each corner and merge the result
  math::Vector3d oldMin = _box.Min();
  math::Vector3d oldMax = _box.Max();
  math::Vector3d newMin(math::MAX_D, math::MAX_D, math::MAX_D);
  math::Vector3d newMax(math::LOW_D, math::LOW_D, math::LOW_D);

  // min min min
  math::Vector3d corner = oldMin;
  auto v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min min max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max max
  corner.Y() = oldMax.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max min
  corner.X() = oldMax.X();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min max
  corner.Y() = oldMin.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  return math::AxisAlignedBox(newMin, newMax);
}

//////////////////////////////////////////////////
void WorldGenerator::SetSeed(int _seed)
{
  this->seed = _seed;
}

//////////////////////////////////////////////////
void WorldGenerator::SetOutputFile(const std::string &_file)
{
  this->outputFile = _file;
}

//////////////////////////////////////////////////
void WorldGenerator::SetMinTileCount(int _tileCount)
{
  this->minTileCount = _tileCount;
}

//////////////////////////////////////////////////
void WorldGenerator::SetEnableGUI(bool _gui)
{
  this->gui = _gui;
}

//////////////////////////////////////////////////
void WorldGenerator::SetWorldName(const std::string &_worldName)
{
  this->worldName = _worldName;
}

//////////////////////////////////////////////////
WorldSection WorldGenerator::CreateWorldSectionFromTile(const std::string &_type,
  const math::Vector3d &_entry, const math::Quaterniond &_rot)
{
  WorldSection s;
  auto it = subt::ConnectionHelper::connectionPoints.find(_type);
  if (it == subt::ConnectionHelper::connectionPoints.end())
  {
    std::cerr << "Unable to find tile type: " << _type << std::endl;
    return s;
  }

  VertexData t;
  t.tileType = _type;
  t.model.SetPose(math::Pose3d(_rot * _entry, _rot));
  s.tiles.push_back(t);

  for (const auto &o : it->second)
  {
    math::Vector3d pt = _rot * (o + _entry);

    // ignore the connection point at zero that we use to connect to previous
    // world section.
    if (pt != math::Vector3d::Zero)
    {
      s.connectionPoints.push_back(std::make_pair(
          pt, math::Quaterniond::Identity));

    }
  }
  return s;
}

//////////////////////////////////////////////////
void WorldGenerator::CreatePrefabWorldSections()
{
  size_t nextId = 0u;
  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave Vertical Shaft Straight Bottom Type A";
    t.model.SetPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Cantilevered Type A";
    t.model.SetPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Straight Top Type A";
    t.model.SetPose(math::Pose3d(75, 0, 25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 02 Type A";
    t.model.SetPose(math::Pose3d(100, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation 02 Type A";
    t.model.SetPose(math::Pose3d(150, 0, 25, 0, 0, -3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Cap Type A";
    t.model.SetPose(math::Pose3d(50, 0, 25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(200, 0, 100), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 0, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, -25), math::Quaterniond::Identity));

    this->worldSections.push_back(s);
  }


  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetPose(math::Pose3d(75, 75, -25, 0, 0, -3.14159));
    s.tiles.push_back(t);


    t.tileType = "Cave Corner 03 Type A";
    t.model.SetPose(math::Pose3d(75, 150, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetPose(math::Pose3d(75, -100, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn Elevation Type A";
    t.model.SetPose(math::Pose3d(175, -50, 0, 0, 0, 0));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(100, 150, -25), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -125, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -75, 25), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave Straight Type A";
    t.model.SetPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetPose(math::Pose3d(75, 0, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetPose(math::Pose3d(75, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetPose(math::Pose3d(50, -100, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 2 Way 01 Type A";
    t.model.SetPose(math::Pose3d(100, -125, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetPose(math::Pose3d(125, -100, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 02 Type A";
    t.model.SetPose(math::Pose3d(125, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, 0), math::Quaterniond(0, 0, IGN_PI/2)));

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetPose(math::Pose3d(50, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 03 Type A";
    t.model.SetPose(math::Pose3d(150, 0, 75, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 75), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -50, 150), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave Straight Type A";
    t.model.SetPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Cavern Type A";
    t.model.SetPose(math::Pose3d(75, 0, -25, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetPose(math::Pose3d(125, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, -25), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -25, -25), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetPose(math::Pose3d(125, -75, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetPose(math::Pose3d(125, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn 01 Type A";
    t.model.SetPose(math::Pose3d(175, -25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    VertexData t;
    s.id = nextId++;

    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetPose(math::Pose3d(125, 25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetPose(math::Pose3d(125, -75, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 01 Type A";
    t.model.SetPose(math::Pose3d(175, 50, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetPose(math::Pose3d(225, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetPose(math::Pose3d(175, -50, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(250, 0, -25), math::Quaterniond::Identity));

    this->worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Split Type A",
        math::Vector3d(0, -50, 0), math::Quaterniond(0, 0, IGN_PI/2)));
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Elevation 01 Type A",
        math::Vector3d(0, -50, -75), math::Quaterniond(0, 0, IGN_PI/2)));
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
      this->CreateWorldSectionFromTile("Cave Elevation Straight Type A",
      math::Vector3d(0, 50, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Straight Type A",
        math::Vector3d(0, 25, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.id = nextId++;
    this->worldSections.push_back(s);
  }
}

//////////////////////////////////////////////////
std::string WorldGenerator::WorldGUIStr() const
{
  std::stringstream ss;
  ss << "    <scene>\n";
  ss << "      <ambient>1.0 1.0 1.0 1.0</ambient>\n";
  ss << "      <background>0.2 0.2 0.2 1.0</background>\n";
  ss << "      <grid>false</grid>\n";
  ss << "      <origin_visual>false</origin_visual>\n";
  ss << "    </scene>\n";
  ss << "    <plugin\n";
  ss << "      filename=\"libignition-gazebo-user-commands-system.so\"\n";
  ss << "      name=\"ignition::gazebo::systems::UserCommands\">\n";
  ss << "    </plugin>\n";
  ss << "    <plugin\n";
  ss << "      filename=\"libignition-gazebo-scene-broadcaster-system.so\"\n";
  ss << "      name=\"ignition::gazebo::systems::SceneBroadcaster\">\n";
  ss << "    </plugin>\n\n";
  ss << "    <gui fullscreen=\"0\">\n";
  ss << "      <!-- 3D scene -->\n";
  ss << "      <plugin filename=\"GzScene3D\" name=\"3D View\">\n";
  ss << "        <ignition-gui>\n";
  ss << "          <title>3D View</title>\n";
  ss << "          <property type=\"bool\" key=\"showTitleBar\">";
  ss <<                "false</property>\n";
  ss << "          <property type=\"string\" key=\"state\">docked</property>\n";
  ss << "        </ignition-gui>\n";
  ss << "        <engine>ogre2</engine>\n";
  ss << "        <scene>scene</scene>\n";
  ss << "        <ambient_light>1.0 1.0 1.0</ambient_light>\n";
  ss << "        <background_color>0.8 0.8 0.8</background_color>\n";
  ss << "        <camera_pose>-6 0 6 0 0.5 0</camera_pose>\n";
  ss << "      </plugin>\n";
  ss << "    </gui>\n\n";

  return ss.str();
}

//////////////////////////////////////////////////
std::string WorldGenerator::WorldTopStr() const
{
  std::stringstream ss;
  ss << "<?xml version=\"1.0\" ?>\n\n";

  ss << "<!--\n";
  ss << "    Generated using world generator:\n";
  ss << "        world_generator_type_a " << (this->gui ? "-g" : "");
  ss <<          " -s " << this->seed;
  ss <<          " -o " << this->outputFile;
  ss <<          " -c " << this->minTileCount;
  ss <<          " -n " << this->worldName << "\n";
  ss << "-->\n\n";

  ss << "<sdf version=\"1.6\">\n";
  ss << "  <world name=\"" << this->worldName << "\">\n\n";

  ss << "    <physics name=\"1ms\" type=\"ode\">\n";
  ss << "      <max_step_size>0.004</max_step_size>\n";
  ss << "      <real_time_factor>1.0</real_time_factor>\n";
  ss << "    </physics>\n\n";

  ss << "    <scene>\n";
  ss << "      <ambient>0.1 0.1 0.1 1.0</ambient>\n";
  ss << "      <background>0 0 0 1.0</background>\n";
  ss << "      <grid>false</grid>\n";
  ss << "      <origin_visual>false</origin_visual>\n";
  ss << "    </scene>\n\n";

  ss << "    <!-- The staging area -->\n";
  ss << "    <include>\n";
  ss << "      <static>true</static>\n";
  ss << "      <name>staging_area</name>\n";
  ss << "      <pose>0 0 0 0 0 0</pose>\n";
  ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  ss <<        "Cave Starting Area Type B</uri>\n";
  ss << "    </include>\n\n";

  ss << "    <!-- The base station -->\n";
  ss << "    <include>\n";
  ss << "      <static>true</static>\n";
  ss << "      <name>base_station</name>\n";
  ss << "      <pose>-8 0 0 0 0 -1.5708</pose>\n";
  ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  ss  <<       "Base Station</uri>\n";
  ss << "    </include>\n\n";

  ss << "    <!-- Fiducial marking the origin for artifacts reports -->\n";
  ss << "    <include>\n";
  ss << "      <name>artifact_origin</name>\n";
  ss << "      <pose>10.0 0.0 0.0 0 0 0</pose>\n";
  ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  ss <<        "Fiducial</uri>\n";
  ss << "    </include>\n\n";

  ss << "    <!-- Tunnel tiles and artifacts -->\n";
  ss << "    <include>\n";
  ss << "      <static>true</static>\n";
  ss << "      <name>tile_1</name>\n";
  ss << "      <pose>37.5 0 0 0 0 1.5708</pose>\n";
  ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  ss <<            "Cave Transition Type A to and from Type B Lights</uri>\n";
  ss << "    </include>\n\n";
  return ss.str();
}

//////////////////////////////////////////////////
std::string WorldGenerator::WorldBottomStr() const
{
  std::stringstream ss;
  ss << "  </world>\n";
  ss << "</sdf>\n";
  return ss.str();
}

//////////////////////////////////////////////////
void WorldGenerator::PreprocessTiles()
{
  // filter tiles
  for (const auto &t : subt::ConnectionHelper::connectionTypes)
  {
    if (t.first.find("Type A") != std::string::npos)
      this->tileConnectionTypes[t.first] = t.second;
  }

  // fetch model and find the mesh file so we can load it and compute
  // bounding box data which are used for intersection checks
  std::cout << "Computing tile bounding box info. "
            << "This may take a while if models need to be downloaded."
            << std::endl;

  fuel_tools::ClientConfig config;
  auto fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  std::string baseUri = "https://fuel.ignitionrobotics.org/openrobotics/models";

  for (const auto &t : tileConnectionTypes)
  {
    // lights just include another model and do not have dae meshes so exclude
    std::string tileType = t.first;
    if (tileType.find("Lights") != std::string::npos)
      continue;

    common::URI modelUri;
    std::string fullUri = common::joinPaths(baseUri, tileType);
    modelUri.Parse(fullUri);

    std::string path;
    auto result = fuelClient->CachedModel(modelUri, path);
    if (result.Type() == fuel_tools::ResultType::FETCH_ERROR)
    {
      std::cout << "Unable to find tile in local cache. Downloading: "
                << tileType << std::endl;

      auto result2 = fuelClient->DownloadModel(modelUri, path);
      if (result2.Type() == fuel_tools::ResultType::FETCH_ERROR)
      {
        std::cerr << "Failed to download tile from fuel: " << tileType
                  << tileType << std::endl;

      }
      continue;
    }

    // find the first dae mesh in the meshes dir
    std::string meshPath = common::joinPaths(path, "meshes");
    std::string resourcePath;
    for (common::DirIter file(meshPath); file != common::DirIter(); ++file)
    {
      std::string current(*file);
      if (current.substr(current.size() - 4) == ".dae")
      {
        resourcePath = current;
      }
    }
    if (resourcePath.empty())
    {
      std::cerr << "Unable to find file with .dae extension in dir: "
                << meshPath << std::endl;
      continue;
    }

    // load dae and compute bbox
    common::MeshManager *meshManager = common::MeshManager::Instance();
    const common::Mesh *mesh = meshManager->Load(resourcePath);
    this->tileBoundingBoxes[tileType] =
        math::AxisAlignedBox(mesh->Min(), mesh->Max());
  }
}

//////////////////////////////////////////////////
WorldSection WorldGenerator::SelectWorldSection()
{
  int r = rand() % worldSections.size();
  return this->worldSections[r];
}

//////////////////////////////////////////////////
bool WorldGenerator::IntersectionCheck(WorldSection &_section,
  const math::Pose3d _pose, const std::vector<WorldSection> &_addedSections)
{
  // do a bounding box intersection check for all tiles in the input world
  // section against all tiles that have been added to the world
  for (const auto &tile : _section.tiles)
  {
    math::Pose3d pose = tile.model.Pose() + _pose;
    math::AxisAlignedBox box = this->tileBoundingBoxes[tile.tileType];
    box = transformAxisAlignedBox(box, pose);
    _section.boundingboxes.push_back(box);

    // store list of connection openings in world frame.
    // If intersection occurs, we check the center of intersection
    // against the opening pos to see if these intersections are due to
    // connection between tiles
    std::vector<math::Vector3d> sectionOpeningsWorld;
    for (const auto &so : _section.connectionPoints)
      sectionOpeningsWorld.push_back(_pose.CoordPositionAdd(so.first));
    sectionOpeningsWorld.push_back(_pose.Pos());

    for (auto &section: _addedSections)
    {
      for (const auto &bbox: section.boundingboxes)
      {
        if (bbox.Intersects(box))
        {
          // when two boxes intersect, the overlapping region is a small box
          // compute this overlapping region
          math::Vector3d min;
          math::Vector3d max;
          min.X() = std::max(box.Min().X(), bbox.Min().X());
          min.Y() = std::max(box.Min().Y(), bbox.Min().Y());
          min.Z() = std::max(box.Min().Z(), bbox.Min().Z());
          max.X() = std::min(box.Max().X(), bbox.Max().X());
          max.Y() = std::min(box.Max().Y(), bbox.Max().Y());
          max.Z() = std::min(box.Max().Z(), bbox.Max().Z());
          math::AxisAlignedBox region(min, max);

          // Get the center of this overlapping region and check whether
          // the overlap occurs at connection points.
          bool overlapAtConnection = false;
          for (const auto &so : sectionOpeningsWorld)
          {
            // if center of overlapping region is not too far away from a
            // connection point, it could be a valid intersection between tiles
            // since the meshes are designed to overlap a little to reduce gaps
            double dist = (region.Center() - so).Length();
            if (dist < 30)
            {
              // make sure the overlapping region is small
              double volume = region.XLength() * region.YLength()
                  * region.ZLength();
              if (volume < 1000)
              {
                overlapAtConnection = true;
                break;
              }
            }
          }

          if (!overlapAtConnection)
            return true;
        }
      }
    }
  }
  return false;
}

//////////////////////////////////////////////////
void WorldGenerator::Generate()
{
  this->PreprocessTiles();
  this->CreatePrefabWorldSections();

  // starting pos specific to type A world
  math::Vector3d startingPos(37.5 + 25.0, 0, 0);

  std::list<std::pair<math::Vector3d, math::Quaterniond>> openings;
  std::list<std::pair<math::Vector3d, math::Quaterniond>> unfilledOpenings;
  openings.push_back(std::make_pair(startingPos, math::Quaterniond::Identity));

  std::vector<WorldSection> addedWorldSections;
  int tileCount = this->minTileCount;
  while (tileCount > 0 && !openings.empty())
  {
    auto o = openings.front();
    openings.pop_front();

    math::Vector3d connectionPoint = o.first;
    math::Quaterniond rot = o.second;

    bool selected = false;
    int attempt = 0;
    WorldSection selectedSection;
    while (!selected && attempt++ < 10)
    {
      // randomly select a world section
      WorldSection s = std::move(this->SelectWorldSection());

      // check if it is a large prefab world section
      // we check to see if the sections has been included or not
      // if so, we want to lower the probablity of it being included again
      if (s.tiles.size() > 1)
      {
        bool exists = false;
        for (const auto &added : addedWorldSections)
        {
          if (added.id == s.id)
          {
            exists = true;
          }
        }
        // if tile exists, set a 30% probablity of it being included again
        if (exists)
        {
          if ((rand() % 10 + 1)> 3)
            continue;
        }
      }

      // transformation from world origin to connection point of the current
      // world section
      math::Pose3d transform(connectionPoint, rot);

      // do bounding box intersection check to prevent overlapping tiles
      if (this->IntersectionCheck(s, transform, addedWorldSections))
        continue;

      // make sure we do not add a section that has zero openings
      // if we have not reached the desired tile count yet.
      bool valid = !(openings.empty() &&
          s.connectionPoints.empty() &&
          tileCount - s.tiles.size() > 0);
      if (!valid)
        continue;

      // add the new section and populate its bounding box data
      s.pose = transform;
      selected = true;
      selectedSection = s;
      addedWorldSections.push_back(selectedSection);

      // add the openings of the new world section to the list
      // be sure to convert to world frame
      for (auto &cp : s.connectionPoints)
      {
        auto tCp = std::make_pair(transform.CoordPositionAdd(cp.first),
            transform.Rot() * cp.second);
        openings.push_back(tCp);
      }

      // update number of tiles that still need to be added
      tileCount -= s.tiles.size();
    }

    // if we fail to add a section, add to the unfilled list
    // so we can put a cap there to block the opening
    if (attempt >= 10)
    {
      unfilledOpenings.push_back(o);
    }
  }

  // add remaining openings to the unfilled list
  for (auto &o : openings)
    unfilledOpenings.push_back(o);

  // begin generating the world sdf
  std::stringstream ss;
  math::AxisAlignedBox worldBBox;
  // tile no starts from 2 (1 is the transition tile)
  int tileNo = 2;
  for (const auto &s : addedWorldSections)
  {
    for (const auto &t : s.tiles)
    {
      std::string name = "tile_" + std::to_string(tileNo++);

      // convert tile pose to world coordinates
      math::Pose3d pose = t.model.Pose() + s.pose;
      std::string uri =
          "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/" +
          t.tileType;

      ss << "    <include>\n";
      ss << "      <static>true</static>\n";
      ss << "      <name>" << name << "</name>\n";
      ss << "      <pose>" << pose << "</pose>\n";
      ss << "      <uri>" << uri << "</uri>\n";
      ss << "    </include>\n\n";
    }

    // Get world bbox
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  capUri += "Cave Cap Type A";

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.first;
    math::Quaterniond rot = uf.second;
    std::string name = "cap_" + std::to_string(capNo++);
    math::Pose3d pose(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);

    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>" << name << "</name>\n";
    ss << "      <pose>" << pose << "</pose>\n";
    ss << "      <uri>" << capUri << "</uri>\n";
    ss << "    </include>\n\n";
  }

  std::string worldStr;
  worldStr += this->WorldTopStr();
  if (this->gui)
    worldStr += this->WorldGUIStr();
  worldStr += ss.str();
  worldStr += this->WorldBottomStr();

  std::ofstream outFile(this->outputFile);
  if (!outFile.is_open())
  {
    std::cerr << "Failed to write to file " << this->outputFile << std::endl;
    return;
  }
  outFile << worldStr;
  outFile.close();

  std::cout << "World Generated: " << this->outputFile << std::endl;
  std::cout << "  Tile count: " << tileNo-1 << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl;
}

//////////////////////////////////////////////////
void printUsage()
{
  std::string usage;
  usage += "Usage: world_generator_type_a [options]\n";
  usage += "Options:\n";
  usage += "    -h\t\t Print this help message\n";
  usage += "    -o <file>\t Output sdf filename\n";
  usage += "    -s <seed>\t Seed\n";
  usage += "    -c <count>\t Min tile count\n";
  usage += "    -n <name>\t World name\n";
  usage += "    -g\t\t Generate sdf with GUI plugin\n";
  std::cout << usage << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printUsage();
    return -1;
  }

  int opt;
  std::string output;
  std::string worldName;
  int seed = 0;
  bool gui = false;
  int tileCount = 10;
  while((opt = getopt(argc, argv, "o:s:c:n:hg")) != -1)
  {
    switch(opt)
    {
      // output
      case 'o':
      {
        output = optarg;
        break;
      }
      // seed
      case 's':
      {
        seed = std::stoi(optarg);
        break;
      }
      // min tile count
      case 'c':
      {
        tileCount = std::stoi(optarg);
        break;
      }
      // world name
      case 'n':
      {
        worldName = optarg;
        break;
      }
      // help
      case 'h':
      {
        printUsage();
        return -1;
      }
      // enable gui?
      case 'g':
      {
        gui = true;
        break;
      }
      default:
        abort();
    }
  }

  srand(seed);

  WorldGenerator wg;
  wg.SetSeed(seed);
  wg.SetOutputFile(output);
  wg.SetMinTileCount(tileCount);
  wg.SetEnableGUI(gui);
  wg.SetWorldName(worldName);
  wg.Generate();

  return 0;
}
