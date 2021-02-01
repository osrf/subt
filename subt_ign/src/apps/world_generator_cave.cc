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

enum TileType
{
  CAVE_TYPE_A = 1,
  CAVE_TYPE_B = 2,
  CAVE_TYPE_TRANSITION = 3
};

/// \brief A connection opening
class ConnectionOpening
{
  /// \brief Position of the  opening
  public: math::Vector3d pos;

  /// \brief Orientation of the opening expressed in rotation from X.
  public: math::Quaterniond rot;

  /// \brief Type of tile for this opening
  public: TileType tileType;
};

/// \brief A section of a world
class WorldSection
{
  /// \brief Unique id
  public: size_t id;

  /// \brief A list of tiles in the world section
  public: std::vector<VertexData> tiles;

  /// \brief A list of connection points and their orientation from +X
  /// This field is manually filled in CreateType[A/B]WorldSections().
  public: std::vector<std::pair<math::Vector3d, math::Quaterniond>>
      connectionPoints;

  /// \brief A list of bounding boxes for the tiles in this world section
  /// This field is auto populated by PreprocessTiles().
  public: std::vector<math::AxisAlignedBox> boundingboxes;

  /// \brief Pose of this world section from world origin
  public: math::Pose3d pose;

  /// \brief Type of tile
  public: TileType tileType;
};

/// \brief Helper class to generate a world from tiles
class WorldGenerator
{
  public: enum WorldType
  {
    CAVE_ANASTOMOTIC = 1,
    CAVE_CURVILINEAR = 2,
    CAVE_RECTILINEAR = 3,
  };

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

  /// \brief Set the world type
  /// \param[in] _type WorldType
  public: void SetWorldType(WorldType _type);

  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Create a collection of prefab world sections from Type A tiles
  /// The worls section are used to create the final generated world
  private: void CreateTypeAWorldSections();

  /// \brief Create a collection of prefab world sections from Type B tiles
  /// The worls section are used to create the final generated world
  private: void CreateTypeBWorldSections();

  /// \brief Create a world section from transitiion tile
  private: void CreateTransitionWorldSection();

  /// \brief Helper function to create a world section from an individual file
  /// \param[in] _type Type of tile
  /// \param[in] _entry Entry position of this tile. This should be one of the
  /// connection points of the tile
  /// \param[in] _rot Rotation to be applied to tile so that its entry point is
  /// in +X direction
  /// \param[in] _tileType Type of tile
  private: WorldSection CreateWorldSectionFromTile(const std::string &_type,
      const math::Vector3d &_entry, const math::Quaterniond &_rot,
      TileType _tileType);

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  private: void LoadTiles();

  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  /// \param[in] _tileType Type of tile to select
  private: WorldSection SelectWorldSection(TileType _tileType);

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

  /// \brief A list of connection points for tiles used by this world generator
  public: std::map<std::string, std::vector<ignition::math::Vector3d>>
      tileConnectionPoints;

  /// \brief A list of tile bounding box data for each tile connection type
  public: std::map<std::string, math::AxisAlignedBox>
      tileBoundingBoxes;

  /// \brief A collection of prefab world sections made of tile type A
  public: std::vector<WorldSection> worldSectionsTypeA;

  /// \brief A collection of prefab world sections made of tile type B
  public: std::vector<WorldSection> worldSectionsTypeB;

  /// \brief World section created from transition tile
  public: WorldSection transitionWorldSection;

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

  /// \brief World type
  private: WorldType worldType = CAVE_ANASTOMOTIC;
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
void WorldGenerator::SetWorldType(WorldType _type)
{
  this->worldType = _type;
}

//////////////////////////////////////////////////
WorldSection WorldGenerator::CreateWorldSectionFromTile(const std::string &_type,
  const math::Vector3d &_entry, const math::Quaterniond &_rot, TileType _tileType)
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
  t.model.SetRawPose(math::Pose3d(_rot * -_entry, _rot));
  s.tiles.push_back(t);

  for (const auto &o : it->second)
  {
    // ignore the connection point at zero that we use to connect to previous
    // world section.
    if (o != _entry)
    {
      math::Vector3d pt = _rot * (-_entry + o);
      math::Quaterniond rot = math::Quaterniond::Identity;
      if (_tileType == CAVE_TYPE_B)
      {
        if (!math::equal(pt.Y(), 0.0))
        {
          if (pt.Y() > 0.0)
            rot = math::Quaterniond(0, 0, IGN_PI * 0.5);
          else
            rot = math::Quaterniond(0, 0, -IGN_PI * 0.5);
        }
        else if (!math::equal(pt.X(), 0.0))
        {
          if (pt.X() < 0.0)
            rot = math::Quaterniond(0, 0, -IGN_PI);
        }
      }

      s.connectionPoints.push_back(std::make_pair(
          pt, rot));

    }
  }
  return s;
}

//////////////////////////////////////////////////
void WorldGenerator::CreateTypeAWorldSections()
{
  static size_t nextId = 0u;
  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Vertical Shaft Straight Bottom Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Cantilevered Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Straight Top Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, 25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 02 Type A";
    t.model.SetRawPose(math::Pose3d(100, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation 02 Type A";
    t.model.SetRawPose(math::Pose3d(150, 0, 25, 0, 0, -3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Cap Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(200, 0, 100), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 0, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, -25), math::Quaterniond::Identity));

    this->worldSectionsTypeA.push_back(s);
  }


  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetRawPose(math::Pose3d(75, 75, -25, 0, 0, -3.14159));
    s.tiles.push_back(t);


    t.tileType = "Cave Corner 03 Type A";
    t.model.SetRawPose(math::Pose3d(75, 150, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, -100, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn Elevation Type A";
    t.model.SetRawPose(math::Pose3d(175, -50, 0, 0, 0, 0));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(100, 150, -25), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -125, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -75, 25), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(50, -100, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 2 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(100, -125, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(125, -100, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 02 Type A";
    t.model.SetRawPose(math::Pose3d(125, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, 0), math::Quaterniond(0, 0, IGN_PI/2)));

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 03 Type A";
    t.model.SetRawPose(math::Pose3d(150, 0, 75, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 75), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -50, 150), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Cavern Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, -25, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetRawPose(math::Pose3d(125, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, -25), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -25, -25), math::Quaterniond(0, 0, -IGN_PI/2)));

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, -75, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(125, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn 01 Type A";
    t.model.SetRawPose(math::Pose3d(175, -25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, 25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, -75, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 01 Type A";
    t.model.SetRawPose(math::Pose3d(175, 50, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetRawPose(math::Pose3d(225, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(175, -50, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(250, 0, -25), math::Quaterniond::Identity));

    this->worldSectionsTypeA.push_back(s);
  }

  // --------------------------
  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Split Type A",
        math::Vector3d(0, 50, 0), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSectionsTypeA.push_back(s);
  }

  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Elevation 01 Type A",
        math::Vector3d(0, 50, 75), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSectionsTypeA.push_back(s);
  }

  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Elevation Straight Type A",
        math::Vector3d(0, -50, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSectionsTypeA.push_back(s);
  }

  {
    WorldSection s = std::move(
        this->CreateWorldSectionFromTile("Cave Straight Type A",
        math::Vector3d(0, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSectionsTypeA.push_back(s);
  }
}

//////////////////////////////////////////////////
void WorldGenerator::CreateTypeBWorldSections()
{
  static size_t nextId = 0u;
  double tileSize = 25.0;
  double halfTileSize = tileSize * 0.5;
  for (const auto &t : this->tileConnectionPoints)
  {
    // only accept Type B tiles
    if (t.first.find("Type B") == std::string::npos)
      continue;

    // filter out Type B to Type A transition tile
    if (t.first.find("Type A") != std::string::npos)
      continue;

    // ignore starting area
    if (t.first.find("Starting") != std::string::npos)
      continue;

    // build cavern manually
    if (t.first.find("Cavern") != std::string::npos)
      continue;

    //
    if (t.first.find("Straight") != std::string::npos ||
        t.first.find("Vertical Shaft") != std::string::npos)
    {
      WorldSection s = std::move(
          this->CreateWorldSectionFromTile(t.first,
          math::Vector3d(0, -halfTileSize, 0), math::Quaterniond(0, 0, -IGN_PI/2),
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      this->worldSectionsTypeB.push_back(s);
    }
    else if (t.first.find("Corner 01") != std::string::npos)
    {
      WorldSection s = std::move(
          this->CreateWorldSectionFromTile(t.first,
          math::Vector3d(0, halfTileSize, 0), math::Quaterniond(0, 0, IGN_PI/2),
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      this->worldSectionsTypeB.push_back(s);
    }
    else if (t.first.find("Corner 02") != std::string::npos)
    {
      WorldSection s = std::move(
          this->CreateWorldSectionFromTile(t.first,
          math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      this->worldSectionsTypeB.push_back(s);
    }
    else if (t.first.find("3 Way") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(halfTileSize, 0, 0),
            math::Quaterniond(0, 0, IGN_PI),
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
    }
    else if (t.first.find("Vertical Shaft") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(halfTileSize, 20, 0),
            math::Quaterniond(0, 0, IGN_PI),
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
    }
    else if (t.first.find("Elevation") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(0, -halfTileSize, 10),
            math::Quaterniond(0, 0, -IGN_PI/2),
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
      {
        WorldSection s = std::move(
            this->CreateWorldSectionFromTile(t.first,
            math::Vector3d(0, halfTileSize, 0),
            math::Quaterniond(0, 0, IGN_PI/2),
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        this->worldSectionsTypeB.push_back(s);
      }
    }
  }

  // build cavern world section
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_B;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Cavern Split 02 Type B";
    t.model.SetRawPose(math::Pose3d(halfTileSize, 0, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave Cavern Split 01 Type B";
    t.model.SetRawPose(math::Pose3d(halfTileSize + tileSize, 0, 0, 0, 0, 0));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(tileSize + tileSize, 0, 0),
        math::Quaterniond::Identity));

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(halfTileSize + tileSize, tileSize, 25),
        math::Quaterniond(0, 0, IGN_PI/2)));

    this->worldSectionsTypeB.push_back(s);
  }
}

//////////////////////////////////////////////////
void WorldGenerator::CreateTransitionWorldSection()
{
  static size_t nextId = 0u;
  std::string type = "Cave Transition Type A to and from Type B";
  this->transitionWorldSection = std::move(
     this->CreateWorldSectionFromTile(type,
      math::Vector3d(0, 25, 0), math::Quaterniond(0, 0, IGN_PI/2),
      CAVE_TYPE_TRANSITION));
  this->transitionWorldSection.tileType = CAVE_TYPE_TRANSITION;
  this->transitionWorldSection.id = nextId++;
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
  ss << "      filename=\"libignition-gazebo-physics-system.so\"\n";
  ss << "      name=\"ignition::gazebo::systems::Physics\">\n";
  ss << "    </plugin>\n";
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
  ss << "    Generated using cave world generator:\n";
  ss << "        world_generator_cave " << (this->gui ? "-g" : "");
  ss <<          " -s " << this->seed;
  ss <<          " -o " << this->outputFile;
  ss <<          " -c " << this->minTileCount;
  ss <<          " -n " << this->worldName;
  ss <<          " -t ";
  switch (this->worldType)
  {
    case CAVE_ANASTOMOTIC:
      ss << "a\n";
      break;
    case CAVE_CURVILINEAR:
      ss << "c\n";
      break;
    case CAVE_RECTILINEAR:
      ss << "r\n";
      break;
    default:
      break;
  }
  ss << "-->\n\n";

  ss << "<sdf version=\"1.6\">\n";
  ss << "  <world name=\"" << this->worldName << "\">\n\n";

  ss << "    <physics name=\"1ms\" type=\"ode\">\n";
  ss << "      <max_step_size>0.004</max_step_size>\n";
  ss << "      <real_time_factor>1.0</real_time_factor>\n";
  ss << "    </physics>\n\n";

  ss << "    <scene>\n";
  ss << "      <ambient>1.0 1.0 1.0 1.0</ambient>\n";
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
void WorldGenerator::LoadTiles()
{
  // filter tiles
  for (const auto &t : subt::ConnectionHelper::connectionPoints)
  {
    // ignore lights
    if (t.first.find("Lights") != std::string::npos)
      continue;

    // ignore 30 deg turns for now
    if (t.first.find("30") != std::string::npos)
      continue;

    if (t.first.find("Type A") != std::string::npos)
    {
      if (this->worldType == CAVE_ANASTOMOTIC ||
          this->worldType == CAVE_CURVILINEAR)
        this->tileConnectionPoints[t.first] = t.second;
    }
    else if (t.first.find("Type B") != std::string::npos)
    {
      if (this->worldType == CAVE_RECTILINEAR ||
          this->worldType == CAVE_CURVILINEAR)
        this->tileConnectionPoints[t.first] = t.second;
    }
  }

  // fetch model and find the mesh file so we can load it and compute
  // bounding box data which are used for intersection checks
  std::cout << "Computing tile bounding box info. "
            << "This may take a while if models need to be downloaded."
            << std::endl;

  fuel_tools::ClientConfig config;
  auto fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  std::string baseUri = "https://fuel.ignitionrobotics.org/OpenRobotics/models";

  for (const auto &t : tileConnectionPoints)
  {
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
    math::AxisAlignedBox bbox = math::AxisAlignedBox(mesh->Min(), mesh->Max());

    // special case for starting area
    if (tileType == "Cave Starting Area Type B")
    {
      bbox = transformAxisAlignedBox(
          bbox, math::Pose3d(0, 0, 0, 0, 0, IGN_PI/2));
    }

    this->tileBoundingBoxes[tileType] = bbox;
  }

  // create prefab world sections
  if (this->worldType == CAVE_ANASTOMOTIC ||
      this->worldType == CAVE_CURVILINEAR)
  {
    this->CreateTransitionWorldSection();
    this->CreateTypeAWorldSections();
  }
  if (this->worldType == CAVE_RECTILINEAR ||
      this->worldType == CAVE_CURVILINEAR)
  {
    this->CreateTypeBWorldSections();
  }
}

//////////////////////////////////////////////////
WorldSection WorldGenerator::SelectWorldSection(TileType _tileType)
{
  if (_tileType == CAVE_TYPE_A)
  {
    int r = rand() % worldSectionsTypeA.size();
    return this->worldSectionsTypeA[r];
  }
  else if (_tileType == CAVE_TYPE_B)
  {
    int r = rand() % worldSectionsTypeB.size();
    return this->worldSectionsTypeB[r];
  }
  else if (_tileType == CAVE_TYPE_TRANSITION)
  {
    return this->transitionWorldSection;
  }
  else
  {
    std::cerr << "Unknown tile type: " << _tileType << std::endl;
    WorldSection s;
    return s;
  }
}

//////////////////////////////////////////////////
bool WorldGenerator::IntersectionCheck(WorldSection &_section,
  const math::Pose3d _pose, const std::vector<WorldSection> &_addedSections)
{
  // do a bounding box intersection check for all tiles in the input world
  // section against all tiles that have been added to the world
  for (const auto &tile : _section.tiles)
  {
    math::Pose3d pose = tile.model.RawPose() + _pose;
    math::AxisAlignedBox box = this->tileBoundingBoxes[tile.tileType];
    box = transformAxisAlignedBox(box, pose);
    // save the bounding box in case we do use this section so that we don't
    // have to compute it again later
    _section.boundingboxes.push_back(box);

    // skip checking if there are only a couple of sections as intersection
    // should not occur. All we need to do is fill the bounding box data above
    if (_addedSections.size() <= 2u)
      continue;

    // first check intersection against starting area
    math::AxisAlignedBox startingAreaBox =
        this->tileBoundingBoxes["Cave Starting Area Type B"];
    if (box.Intersects(startingAreaBox))
    {
      return true;
    }

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

              double maxOverlapVolume = 1000;
              if (section.tileType == CAVE_TYPE_B)
                maxOverlapVolume = 1800;
              if (volume < maxOverlapVolume)
              {
                overlapAtConnection = true;
                break;
              }
            }
          }

          if (!overlapAtConnection)
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

//////////////////////////////////////////////////
void WorldGenerator::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond::Identity;
  op.pos += math::Vector3d(12.5, 0, 0);
  op.tileType = CAVE_TYPE_B;
  openings.push_back(op);

  std::vector<WorldSection> addedWorldSections;
  int tileCount = this->minTileCount;

  // loop through all openings in the world, starting from the staging area,
  // and add tiles to these connection points until we reach the specified
  // tile count
  while (tileCount > 0 && !openings.empty())
  {
    auto o = openings.front();
    openings.pop_front();

    math::Vector3d connectionPoint = o.pos;
    math::Quaterniond rot = o.rot;
    TileType tileType = o.tileType;

    bool selected = false;
    WorldSection selectedSection;

    // Set max number of attempts at adding a tile to the current connection
    // point before giving up. Failure to add a tile is mostly due to
    // intersection with existing tiles in the world.
    int attempt = 0;
    int maxAttempt = 20;
    while (!selected && attempt++ < maxAttempt)
    {
      // randomly select a world section
      TileType tileTypeToSelect = tileType;

      // set a 20% probablity of including a transition tile for
      // curvilinear worlds
      if (this->worldType == WorldType::CAVE_CURVILINEAR)
      {
        if ((rand() % 10 + 1) > 8)
          tileTypeToSelect = CAVE_TYPE_TRANSITION;
      }
      // first tile in anastomotic cave must be a transition tile
      else if (this->worldType == WorldType::CAVE_ANASTOMOTIC)
      {
        if (addedWorldSections.empty())
          tileTypeToSelect = CAVE_TYPE_TRANSITION;
      }

      WorldSection s = std::move(this->SelectWorldSection(tileTypeToSelect));

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
            break;
          }
        }
        // if section exists, set a 70% probablity of skipping this section
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
      {
        continue;
      }

      // make sure we do not add a section that has zero openings
      // if we have not reached the desired tile count yet.
      bool valid = !(openings.empty() &&
          s.connectionPoints.empty() &&
          tileCount - s.tiles.size() > 0);
      if (!valid)
        continue;

      // set the world pose of the new section
      s.pose = transform;

      // if it is a transition tile, we need to make sure connection points
      // match. The transition world section created has Type A connection
      // point at +x and Type B connection point at -x
      bool flip = false;
      if (tileTypeToSelect == CAVE_TYPE_TRANSITION)
      {
        // if the tile to connect to is Type A, we need to flip the transition
        // tile by 180 degrees
        if (tileType == CAVE_TYPE_A)
        {
          auto &transitionTile = s.tiles[0].model;
          transitionTile.SetRawPose(math::Pose3d(transitionTile.RawPose().Pos(),
              math::Quaterniond(0, 0, IGN_PI) * transitionTile.RawPose().Rot()));
          flip = true;
        }
      }

      // add the new section
      selected = true;
      selectedSection = s;
      addedWorldSections.push_back(selectedSection);

      // add the openings of the new world section to the list
      // be sure to convert to world frame
      for (auto &cp : s.connectionPoints)
      {
        ConnectionOpening op;
        op.pos = transform.CoordPositionAdd(cp.first);
        op.rot = transform.Rot() * cp.second;

        // mark the tile type for the opening of the transition tile
        // based on the tile type the opening connects to.
        if (s.tileType == CAVE_TYPE_TRANSITION)
        {
          if (flip)
            op.tileType = CAVE_TYPE_B;
          else
            op.tileType = CAVE_TYPE_A;
        }
        else
        {
          op.tileType = s.tileType;
        }
        openings.push_back(op);
      }

      // update number of tiles that still need to be added
      tileCount -= s.tiles.size();
    }

    // if we fail to add a section, add to the unfilled list
    // so we can put a cap there to block the opening
    if (attempt >= maxAttempt)
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
  int tileNo = 0;
  int caveTypeACount = 0;
  int caveTypeBCount = 0;
  int caveTransitionCount = 0;
  for (const auto &s : addedWorldSections)
  {
    for (const auto &t : s.tiles)
    {
      std::string name = "tile_" + std::to_string(++tileNo);

      // convert tile pose to world coordinates
      math::Pose3d pose = t.model.RawPose() + s.pose;
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

    // keep track of tile count for each tile type
    if (s.tileType == CAVE_TYPE_A)
      caveTypeACount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_B)
      caveTypeBCount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_TRANSITION)
      caveTransitionCount++;

    // update world bbox info
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  std::string capUriTypeA = capUri + "Cave Cap Type A";
  std::string capUriTypeB = capUri + "Cave Cap Type B";

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);
    math::Pose3d pose;
    std::string uri;
    if (uf.tileType == CAVE_TYPE_A)
    {
      uri = capUriTypeA;
      // the cap is placed at opening
      pose = math::Pose3d(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);
    }
    else if (uf.tileType == CAVE_TYPE_B)
    {
      uri = capUriTypeB;
      // the cap origin is at an offset and so it needs to be placed at
      // center of the next tile position instead of where the opening is
      pose = math::Pose3d(pos + rot*math::Vector3d(12.5, 0, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>" << name << "</name>\n";
    ss << "      <pose>" << pose << "</pose>\n";
    ss << "      <uri>" << uri << "</uri>\n";
    ss << "    </include>\n\n";
  }

  // assemble the sdf string
  std::string worldStr;
  worldStr += this->WorldTopStr();
  if (this->gui)
    worldStr += this->WorldGUIStr();
  worldStr += ss.str();
  worldStr += this->WorldBottomStr();

  // output to file
  std::ofstream outFile(this->outputFile);
  if (!outFile.is_open())
  {
    std::cerr << "Failed to write to file " << this->outputFile << std::endl;
    return;
  }
  outFile << worldStr;
  outFile.close();

  // print out some stats
  std::cout << "World Generated: " << this->outputFile << std::endl;
  std::cout << "  Total Tile count: " << tileNo << std::endl;
  std::cout << "    Type A Tile count: " << caveTypeACount << std::endl;
  std::cout << "    Type B Tile count: " << caveTypeBCount<< std::endl;
  std::cout << "    Transition Tile count: " << caveTransitionCount << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl;
}

//////////////////////////////////////////////////
void printUsage()
{
  std::string usage;
  usage += "Usage: world_generator_cave [options]\n";
  usage += "Options:\n";
  usage += "    -h\t\t Print this help message\n";
  usage += "    -o <file>\t Output sdf filename\n";
  usage += "    -s <seed>\t Seed\n";
  usage += "    -c <count>\t Min tile count\n";
  usage += "    -n <name>\t World name\n";
  usage += "    -t <type>\t Cave Type:\n";
  usage += "             \t    'anastomotic' or 'a',\n";
  usage += "             \t    'curvilinear' or 'c',\n";
  usage += "             \t    'rectilinear' or 'r'\n";
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
  std::string worldType = "anastomotic";
  int seed = 0;
  bool gui = false;
  int tileCount = 10;
  while((opt = getopt(argc, argv, "t:o:s:c:n:hg")) != -1)
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
      // cave type
      case 't':
      {
        worldType = optarg;
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
        printUsage();
        return -1;
    }
  }

  srand(seed);

  WorldGenerator wg;
  wg.SetSeed(seed);
  wg.SetOutputFile(output);
  wg.SetMinTileCount(tileCount);
  wg.SetEnableGUI(gui);
  wg.SetWorldName(worldName);

  WorldGenerator::WorldType wt;
  if (worldType == "a" || worldType == "anastomotic")
  {
    wt = WorldGenerator::WorldType::CAVE_ANASTOMOTIC;
  }
  else if (worldType == "c" || worldType == "curvilinear")
  {
    wt = WorldGenerator::WorldType::CAVE_CURVILINEAR;
  }
  else if (worldType == "r" || worldType == "rectilinear")
  {
    wt = WorldGenerator::WorldType::CAVE_RECTILINEAR;
  }
  else
  {
    std::cerr << "One world type in [anastomoatic, curvilinear, rectilinear] must be specified\n";
    return 1;
  }

  wg.SetWorldType(wt);
  wg.Generate();

  return 0;
}
