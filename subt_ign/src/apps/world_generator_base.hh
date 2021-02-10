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
#ifndef WORLD_GENERATOR_BASE_H
#define WORLD_GENERATOR_BASE_H

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

enum SubWorldType
{
  CAVE_ANASTOMOTIC = 1,
  CAVE_CURVILINEAR = 2,
  CAVE_RECTILINEAR = 3,
  URBAN_SUBWAY = 4,
  URBAN_BUILDING = 5,
  URBAN_MIXED_STRUCTURE = 6
};

enum TileType 
{
  NONE = 0, // Expand to sub-domains if meshes types vary in tunnel
  CAVE_TYPE_A = 1,
  CAVE_TYPE_B = 2,
  CAVE_TYPE_TRANSITION = 3,
  URBAN_S = 4,
  URBAN_B = 5,
  URBAN_MS = 6
  
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

//////////////////////////////////////////////////
class WorldGeneratorBase
{
  /// \brief Set the output sdf file to be saved
  /// \param[in] _file Path to save the file
  public: void SetOutputFile(const std::string &_file);

  /// \brief Set whether to enable GUI in the generate world sdf
  /// \param[in] _gui True to enable gui
  public: void SetEnableGUI(bool _gui);

  /// \brief Set the world name
  /// \param[in] _worldName World name
  public: void SetWorldName(const std::string &_worldName);

  /// \brief Set the world type 
  /// \param[in] _worldType World type
  public: void SetWorldType(const std::string &_worldType);

  /// \brief Set the sub world type
  /// \param[in] _subWorldType Sub-world type
  public: void SetSubWorldType(SubWorldType _subWorldType);

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  public: virtual void LoadTiles() = 0;

  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  /// \param[in] _tileType Type of tile to select
  public: virtual WorldSection SelectWorldSection(TileType &_tileType) = 0;

  /// \brief Intersection check between a section to be added to the world
  /// and all the sections that have already been added.
  /// \param[in] _section Section to add to the world.
  /// \param[in] _pose Pose to add the section
  /// \param[in] _addedSections Sections already in the world
  protected: bool IntersectionCheck(WorldSection &_section,
      const math::Pose3d _pose,
      const std::vector<WorldSection> &_addedSections);

  /// \brief Get GUI plugin string
  protected: std::string WorldGUIStr() const;

  /// \brief Get the bottom part of the world sdf string
  protected: std::string WorldBottomStr() const;

  /// \brief Verify the correct orientation of a transition WorldSection
  protected: virtual bool CorrectTransitionWorldPose(WorldSection &_s, TileType &_type);

  /// \brief A list of connection points for tiles used by this world generator
  protected: std::map<std::string, std::vector<ignition::math::Vector3d>>
      tileConnectionPoints;

  /// \brief A list of tile bounding box data for each tile connection type
  protected: std::map<std::string, math::AxisAlignedBox>
      tileBoundingBoxes;

  /// \brief File to save the world sdf to
  protected: std::string outputFile = "out.sdf";

  /// \brief Flag to enable gui
  protected: bool gui = false;

  /// \brief Name of generated world
  protected: std::string worldName = "world_name";

  /// \brief Type of generated world
  protected: std::string worldType = "";

  /// \brief Potential type of structure to generated world
  protected: SubWorldType subWorldType;

  /// \brief A collection of prefab world sections made of tile
  protected: std::vector<WorldSection> worldSections;

};

//////////////////////////////////////////////////
class WorldGenerator : public WorldGeneratorBase
{

  /// \brief Generate the world sdf
  public: virtual void Generate() = 0;

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles();
  
  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  protected: WorldSection SelectWorldSection(TileType &_tileType);

  /// \brief Set the seed for generating random numbers
  /// \param[in] _seed Seed to set
  public: void SetSeed(int _seed);

  /// \brief Set the min number of tiles to include in the generated world
  /// \param-in] _tileCount Min number of tiles
  public: void SetMinTileCount(int _tileCount);

  /// \brief A reference of prefab world sections chosen for world
  protected: std::vector<WorldSection> addedWorldSections;

  /// \brief Seed
  protected: int seed = 0;

  /// \brief Min number of tiles in include in the world
  protected: int minTileCount = 10;

};

//////////////////////////////////////////////////
class WorldGeneratorDebug : public WorldGeneratorBase
{
  /// \brief Generate the world sdf
  public: virtual void Generate() = 0;

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles();
  
  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  protected: WorldSection SelectWorldSection(TileType &_tileType);

  /// \brief Set the tile names
  /// \param[in] _tiles Name of tile to generate worlds from
  public: void SetTileName(const std::string &_tile);

  /// \brief Name of tile of interest
  protected: std::string tileName = "";

};
}

#endif /*WORLD_GENERATOR_BASE_H*/