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

#include "world_generator_base.hh"
#include "world_generator_utils.hh"

//////////////////////////////////////////////////
void WorldGeneratorBase::SetOutputFile(const std::string &_file)
{
    this->outputFile = _file;
}

//////////////////////////////////////////////////
void WorldGeneratorBase::SetEnableGUI(bool _gui)
{
    this->gui = _gui;
}

//////////////////////////////////////////////////
void WorldGeneratorBase::SetWorldName(const std::string &_worldName)
{
    this->worldName = _worldName;
}

//////////////////////////////////////////////////
void WorldGeneratorBase::SetWorldType(const std::string &_worldType)
{
    this->worldType = _worldType;
}

//////////////////////////////////////////////////
void WorldGeneratorBase::SetSubWorldType(SubWorldType _subWorldType)
{
    this->subWorldType = _subWorldType;
}

//////////////////////////////////////////////////
bool WorldGeneratorBase::IntersectionCheck(WorldSection &_section,
    const math::Pose3d _pose,
    const std::vector<WorldSection> &_addedSections)
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
    std::string startingArea = "";
    if (this->worldType == "Tunnel") {startingArea = "subt_tunnel_staging_area";}
    else if (this->worldType == "Urban") {startingArea = "Urban Starting Area";}
    else if (this->worldType == "Cave") {startingArea = "Cave Starting Area Type B";}
    else 
    {
      std::cout << "Unknown world type: " << this->worldType << std::endl;
      return false;
    }
    math::AxisAlignedBox startingAreaBox =
        this->tileBoundingBoxes[startingArea];
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
            if (this->worldType == "Cave")
            {
              if (dist < 15)
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
            if (this->worldType == "Tunnel")
            {
              if (dist < 5)
              {
                // make sure the overlapping region is small
                double volume = region.XLength() * region.YLength()
                    * region.ZLength();
                double maxOverlapVolume = 100;
                if (volume < maxOverlapVolume)
                {
                  overlapAtConnection = true;
                  break;
                }
              }
            }
            else if (this->worldType == "Urban")
            {

            }
            // TODO for Urban circuit
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
std::string WorldGeneratorBase::WorldBottomStr() const
{
  std::stringstream ss;
  ss << "  </world>\n";
  ss << "</sdf>\n";
  return ss.str();
}

//////////////////////////////////////////////////
std::string WorldGeneratorBase::WorldGUIStr() const
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
bool WorldGeneratorBase::CorrectTransitionWorldPose(WorldSection &_s, TileType &_tileType)
{
  if (_s.tileType != _tileType)
    return false;
  return true;
}

//////////////////////////////////////////////////
void WorldGenerator::SetSeed(int _seed)
{
  this->seed = _seed;
}

//////////////////////////////////////////////////
void WorldGenerator::SetMinTileCount(int _tileCount)
{
  this->minTileCount = _tileCount;
}

//////////////////////////////////////////////////
void WorldGenerator::LoadTiles()
{
  // filter tiles
  for (const auto &t : subt::ConnectionHelper::connectionPoints)
  {
    // Add starting area if tunnel circuit
    if (this->worldType == "Tunnel" && t.first.find("tunnel") != std::string::npos)
    {
      this->tileConnectionPoints[t.first] = t.second;
      continue;
    }

    // ignore lights
    if (t.first.find("Lights") != std::string::npos)
      continue;

    // ignore 30 deg turns for now
    if (t.first.find("30") != std::string::npos)
      continue;

    // Ignore disjointed tunnel tiles
    if (t.first.find("Tunnel Tile 3") != std::string::npos)
      continue;

    if (t.first.find("Tunnel Tile 4") != std::string::npos)
      continue;

    // Ignore all tiles not from world type
    if (t.first.find(this->worldType) != std::string::npos)
      this->tileConnectionPoints[t.first] = t.second;
  }

  // fetch model and find the mesh file so we can load it and compute
  // bounding box data which are used for intersection checks
  std::cout << "Computing tile bounding box info. "
            << "This may take a while if models need to be downloaded."
            << std::endl;

  fuel_tools::ClientConfig config;
  auto fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  std::string baseUri = "https://fuel.ignitionrobotics.org/openrobotics/models";

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
      else if (current.substr(current.size() - 4) == ".obj")
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
    if (tileType == "Urban Starting Area")
    {
      bbox = transformAxisAlignedBox(
          bbox, math::Pose3d(0, 0, 0, 0, 0, IGN_PI/2));
    }
    if (tileType == "subt_tunnel_staging_area")
    {
      // Need to correct tunnel starting area bounding box (currently like a plane)
      // TODO Adjust common::Mesh to correct mesh min/max values
      bbox = math::AxisAlignedBox(-13.75, 12.5, 0, 10, -5, 20); //   
      bbox = transformAxisAlignedBox(
          bbox, math::Pose3d(0, 0, 0, 0, 0, IGN_PI/2));
    }
    this->tileBoundingBoxes[tileType] = bbox;
  }
}

//////////////////////////////////////////////////
WorldSection WorldGenerator::SelectWorldSection(TileType &_tileType)
{
  int r = rand() % this->worldSections.size();
  // TODO remove maxAttempts <- while loop should almost surely not be infinitely recursive 
  int maxAttempts = 10;
  WorldSection s = this->worldSections[r];

  while(s.tileType != _tileType && maxAttempts-- > 0)
  {
    r = rand() % this->worldSections.size();
    s = this->worldSections[r];
  }
  
  return s; 
}

//////////////////////////////////////////////////
void WorldGeneratorDebug::SetTileName(const std::string &_tile)
{
  this->tileName = _tile;
}

//////////////////////////////////////////////////
void WorldGeneratorDebug::LoadTiles()
{
  // filter tiles
  for (const auto &t : subt::ConnectionHelper::connectionPoints)
  {
    // Add starting area if tunnel circuit
    if (this->worldType == "Tunnel" && t.first.find("tunnel") != std::string::npos)
    {
      std::cout << "Loading: " << t.first << std::endl;
      this->tileConnectionPoints[t.first] = t.second;
    }

    // If Cave Type A tile add transition tile
    if (this->tileName.find("Type A") != std::string::npos && t.first.find("Transition") != std::string::npos)
    {
      std::cout << "Loading: " << t.first << std::endl;
      this->tileConnectionPoints[t.first] = t.second;
    }

    // If Urban Subway tile add transition tile
    // TODO
    // if (this->worldType == "Urban" && t.first.find(""))
    // Ignore all tiles except needed tile name
    if (this->tileName.find(t.first) == std::string::npos)
      continue;

    std::cout << "Loading: " << t.first << std::endl;
    this->tileConnectionPoints[t.first] = t.second; 
  }

  // fetch model and find the mesh file so we can load it and compute
  // bounding box data which are used for intersection checks
  std::cout << "Computing tile bounding box info. "
            << "This may take a while if models need to be downloaded."
            << std::endl;

  fuel_tools::ClientConfig config;
  auto fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  std::string baseUri = "https://fuel.ignitionrobotics.org/openrobotics/models";

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
      else if (current.substr(current.size() - 4) == ".obj")
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
    // TODO VERIFY
    if (tileType == "Urban Starting Area")
    {
      bbox = transformAxisAlignedBox(
          bbox, math::Pose3d(0, 0, 0, 0, 0, IGN_PI/2));
    }
    if (tileType == "subt_tunnel_staging_area")
    {
      bbox = transformAxisAlignedBox(
          bbox, math::Pose3d(0, 0, 0, 0, 0, IGN_PI/2));
    }

    this->tileBoundingBoxes[tileType] = bbox;
  }
}

//////////////////////////////////////////////////
WorldSection WorldGeneratorDebug::SelectWorldSection(TileType &_tileType)
{
  WorldSection s;
  for (const auto &_s : this->worldSections)
  {
    if (_s.tiles[0].tileType == this->tileName)
      s = std::move(_s);
  }
  if (s.tileType != _tileType)
    std::cout << "WARNING: Mis-matching TileType\t" << _tileType << " != " << s.tileType << std::endl;
  return s;
}
