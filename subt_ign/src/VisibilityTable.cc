/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <utility>
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_ign/Config.hh>
#include <subt_ign/SimpleDOTParser.hh>
#include <subt_ign/VisibilityTable.hh>

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
VisibilityTable::VisibilityTable()
{
};

//////////////////////////////////////////////////
bool VisibilityTable::Load(const std::string &_worldName, bool _loadLUT)
{
  std::string worldsDirectory = SUBT_INSTALL_WORLD_DIR;
  this->worldName = _worldName;

  // Modifications for the tunnel circuit.
  const std::string tunnelPrefix = "tunnel_circuit_";
  const std::string urbanPrefix = "urban_circuit_";
  const std::string cavePrefix = "cave_circuit_";
  if (0 == this->worldName.compare(0, tunnelPrefix.size(), tunnelPrefix))
  {
    std::string suffix = this->worldName.substr(tunnelPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "tunnel_circuit", suffix);
    }
  }
  else if (this->worldName.find(urbanPrefix) != std::string::npos)
  {
    std::string suffix = this->worldName.substr(urbanPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "urban_circuit", suffix);
    }
  }
  else if (this->worldName.find(cavePrefix) != std::string::npos)
  {
    std::string suffix = this->worldName.substr(cavePrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "cave_circuit", suffix);
    }
  }
  else if (this->worldName.find("simple") == std::string::npos)
  {
    ignerr << "Unable to determine circuit number from["
      << this->worldName << "].\n";
  }

  this->worldPath = ignition::common::joinPaths(
    worldsDirectory, _worldName + ".sdf");

  this->graphPath = ignition::common::joinPaths(
    worldsDirectory, _worldName + ".dot");

  this->lutPath = ignition::common::joinPaths(
    worldsDirectory, _worldName + ".dat");

  // Parse the .dot file and populate the world graph.
  if (!this->PopulateVisibilityGraph(graphPath))
  {
    ignerr << "Unable to populate visibility graph using path["
      << graphPath << "]\n";
    return false;
  }

  if (_loadLUT)
    return this->LoadLUT();

  return true;
}

//////////////////////////////////////////////////
bool VisibilityTable::LoadLUT()
{
  std::ifstream in;
  in.open(this->lutPath);
  if (!in.is_open())
  {
    std::cerr << "[VisibilityTable] Unable to find file ["
              << this->lutPath << "]" << std::endl;
    return false;
  }

  // First, load the number of entries.
  uint64_t numEntries;
  in.read(reinterpret_cast<char*>(&numEntries), sizeof(numEntries));

  // Now save all entries.
  for (auto i = 0u; i < numEntries; ++i)
  {
    int32_t x;
    int32_t y;
    int32_t z;
    uint64_t vertexId;

    in.read(reinterpret_cast<char*>(&x), sizeof(x));
    in.read(reinterpret_cast<char*>(&y), sizeof(y));
    in.read(reinterpret_cast<char*>(&z), sizeof(z));
    in.read(reinterpret_cast<char*>(&vertexId), sizeof(vertexId));
    this->vertices[std::make_tuple(x, y, z)] = vertexId;
  }

  this->PopulateVisibilityInfo();

  return true;
}

//////////////////////////////////////////////////
double VisibilityTable::Cost(const ignition::math::Vector3d &_from,
  const ignition::math::Vector3d &_to) const
{
  int32_t x = std::round(_from.X());
  int32_t y = std::round(_from.Y());
  int32_t z = std::round(_from.Z());
  auto sampleFrom = std::make_tuple(x, y, z);

  x = std::round(_to.X());
  y = std::round(_to.Y());
  z = std::round(_to.Z());
  auto sampleTo = std::make_tuple(x, y, z);

  uint64_t from = std::numeric_limits<uint64_t>::max();
  auto it = this->vertices.find(sampleFrom);
  if (it != this->vertices.end())
    from = it->second;

  uint64_t to = std::numeric_limits<uint64_t>::max();
  it = this->vertices.find(sampleTo);
  if (it != this->vertices.end())
    to = it->second;

  auto key = std::make_pair(from, to);
  auto itVisibility = this->visibilityInfo.find(key);
  if (itVisibility == this->visibilityInfo.end())
    return std::numeric_limits<double>::max();

  // The cost.
  return itVisibility->second;
}

//////////////////////////////////////////////////
void VisibilityTable::Generate()
{
  this->CreateWorldSegments();

  ignmsg << "Building LUT" << std::endl;
  this->BuildLUT();

  this->WriteOutputFile();
}

//////////////////////////////////////////////////
void VisibilityTable::SetModelBoundingBoxes(
    const std::map<std::string, ignition::math::AxisAlignedBox> &_boxes)
{
  this->bboxes = _boxes;
}

//////////////////////////////////////////////////
const std::map<std::tuple<int32_t, int32_t, int32_t>, uint64_t>
  &VisibilityTable::Vertices() const
{
  return this->vertices;
}

//////////////////////////////////////////////////
uint64_t VisibilityTable::Index(const ignition::math::Vector3d &_position) const
{
  for (auto const segment : this->worldSegments)
  {
    const ignition::math::AxisAlignedBox &box = segment.first;
    if (box.Contains(_position))
      return segment.second;
  }

  return std::numeric_limits<uint64_t>::max();
}

//////////////////////////////////////////////////
bool VisibilityTable::PopulateVisibilityGraph(const std::string &_graphFilename)
{
  std::filebuf fb;
  if (!fb.open(_graphFilename, std::ios::in))
  {
    std::cerr << "Unable to read [" << _graphFilename << "] file" << std::endl;
    return false;
  }

  std::istream is(&fb);
  SimpleDOTParser dotParser;
  if (!dotParser.Parse(is))
    return false;

  this->visibilityGraph = dotParser.Graph();

  return true;
}

//////////////////////////////////////////////////
void VisibilityTable::PopulateVisibilityInfo()
{
  // Cached it.
  if (!this->visibilityInfoWithoutRelays.empty())
  {
    this->visibilityInfo = this->visibilityInfoWithoutRelays;
    return;
  }

  // Get the list of vertices Id.
  auto vertexIds = this->visibilityGraph.Vertices();

  // Get the cost between all vertex pairs.
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    auto result = ignition::math::graph::Dijkstra(this->visibilityGraph, id1);
    for (const auto to : result)
    {
      auto id2 = to.first;
      double cost = to.second.first;
      this->visibilityInfo[std::make_pair(id1, id2)] = cost;
    }
  }

  this->visibilityInfoWithoutRelays = this->visibilityInfo;
}

//////////////////////////////////////////////////
void VisibilityTable::PopulateVisibilityInfo(
  const std::set<ignition::math::Vector3d> &_relayPoses)
{
  // Convert poses to vertices.
  std::set<ignition::math::graph::VertexId> relays;
  for (const auto pose : _relayPoses)
  {
    int32_t x = std::round(pose.X());
    int32_t y = std::round(pose.Y());
    int32_t z = std::round(pose.Z());
    auto vertexId = std::make_tuple(x, y, z);

    auto it = this->vertices.find(vertexId);
    if (it != this->vertices.end())
      relays.insert(it->second);
  }

  // Compute the cost of all routes without considering relays.
  this->PopulateVisibilityInfo();

  // Update the cost of all routes considering relays.
  VisibilityInfo visibilityInfoWithRelays;
  std::set<ignition::math::graph::VertexId> visited;
  auto vertexIds = this->visibilityGraph.Vertices();
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    for (const auto to : vertexIds)
    {
      auto id2 = to.first;
      this->PopulateVisibilityInfoHelper(
        relays, id1, id2, visited, visibilityInfoWithRelays);
      visited = {};
    }
  }

  this->visibilityInfo = visibilityInfoWithRelays;
}

//////////////////////////////////////////////////
bool VisibilityTable::PopulateVisibilityInfoHelper(
  const std::set<ignition::math::graph::VertexId> &_relays,
  const ignition::math::graph::VertexId &_from,
  const ignition::math::graph::VertexId &_to,
  std::set<ignition::math::graph::VertexId> &_visited,
  VisibilityInfo &_visibilityInfoWithRelays)
{
  auto srcToDstCostIt = this->visibilityInfo.find(std::make_pair(_from, _to));
  if (srcToDstCostIt == this->visibilityInfo.end())
    return false;

  double srcToDstCost = srcToDstCostIt->second;
  double bestRouteCost = std::numeric_limits<double>::max();

  if (_from != _to)
  {
    for (const auto i : _relays)
    {
      if (_from == i || _to == i || _visited.find(i) != _visited.end())
        continue;

      // I can't reach the relay.
      auto srcToRelayIt = this->visibilityInfo.find(std::make_pair(_from, i));
      if (srcToRelayIt == this->visibilityInfo.end())
        continue;

      // I can reach the relay with this cost.
      double srcToRelayCost = srcToRelayIt->second;

      // Route not cached.
      auto relayToDstIt =
        _visibilityInfoWithRelays.find(std::make_pair(i, _to));
      if (relayToDstIt == _visibilityInfoWithRelays.end())
      {
        // Mark this relay as visited to avoid infinite recursion.
        _visited.insert(i);

        // Evaluate the route from the relay to the destination.
        this->PopulateVisibilityInfoHelper(
          _relays, i, _to, _visited, _visibilityInfoWithRelays);
      }

      // Do we now have a route from the relay to the destination?
      double relayToDstCost = std::numeric_limits<double>::max();
      relayToDstIt = _visibilityInfoWithRelays.find(std::make_pair(i, _to));
      if (relayToDstIt != _visibilityInfoWithRelays.end())
        relayToDstCost = relayToDstIt->second;

      // The cost of a route is the cost of its biggest hop.
      double currentRouteCost = std::max(srcToRelayCost, relayToDstCost);

      // Among all the routes found going through relays,
      // select the one with lowest cost.
      bestRouteCost = std::min(bestRouteCost, currentRouteCost);
    }
  }

  // Among all posible routes, select the one with lowest cost.
  _visibilityInfoWithRelays[std::make_pair(_from, _to)] =
    std::min(srcToDstCost, bestRouteCost);
  _visibilityInfoWithRelays[std::make_pair(_to, _from)] =
    std::min(srcToDstCost, bestRouteCost);

  return true;
}

//////////////////////////////////////////////////
void VisibilityTable::CreateWorldSegments()
{
  // Get the list of vertices Id.
  auto vertexIds = this->visibilityGraph.Vertices();

  for (const auto from : vertexIds)
  {
    std::string data = from.second.get().Data();
    // Remove the quotes.
    data.erase(0, 1);
    data.pop_back();

    auto fields = ignition::common::split(data, "::");
    if (fields.size() != 3u)
    {
      std::cerr << "Incorrect format. Expecting <ID::type::name> and found ["
                << data << "]. Ignoring vertex" << std::endl;
      continue;
    }
    std::string modelName = fields.at(2);

    auto bboxIt = this->bboxes.find(modelName);
    if (bboxIt == this->bboxes.end())
    {
      // For backward compatibility
      // The dot graph generated by tile_tsv.py names the staging area
      // "BaseStation" but in ignition sdf world file it is named "staging_area"
      if (modelName == "BaseStation")
        bboxIt = this->bboxes.find("staging_area");

      if (bboxIt == this->bboxes.end())
      {
        ignerr << "Unable to find bounding box info for model: ["
               << modelName << "]. Ignoring vertex" << std::endl;
        continue;
      }
    }
    this->worldSegments.push_back(
        std::make_pair(bboxIt->second, from.first));
  }
}

//////////////////////////////////////////////////
void VisibilityTable::BuildLUT()
{
  auto zrange = this->kMaxZ - this->kMinZ;
  int counter = 0;
  for (auto z = this->kMinZ; z <= this->kMaxZ; ++z)
  {
    for (auto y = this->kMinY; y <= this->kMaxY; ++y)
    {
      for (auto x = this->kMinX; x <= this->kMaxX; ++x)
      {
        ignition::math::Vector3d position(x, y, z);
        auto index = this->Index(position);
        if (index != std::numeric_limits<uint64_t>::max())
          this->vertices[std::make_tuple(x, y, z)] = index;
      }
    }

    // Display the percentage.
    std::cout << std::fixed << std::setprecision(0)
              << std::min(100.0, 100.0 * ++counter / zrange) << " %\r";
    std::cout.flush();
  }
  std::cout << std::endl;
}

//////////////////////////////////////////////////
void VisibilityTable::WriteOutputFile()
{
  std::fstream out(this->lutPath, std::ios::out | std::ios::binary);
  if (!out)
  {
    std::cerr << "Unable to create [" << this->lutPath << "] file" << std::endl;
    return;
  }

  uint64_t numEntries = this->vertices.size();

  // First, save the number of entries.
  out.write(reinterpret_cast<const char*>(&numEntries), sizeof(numEntries));

  // Now save all entries.
  for (const auto entry : this->vertices)
  {
    int32_t x = std::get<0>(entry.first);
    int32_t y = std::get<1>(entry.first);
    int32_t z = std::get<2>(entry.first);
    uint64_t vertexId = entry.second;
    out.write(reinterpret_cast<const char*>(&x), sizeof(x));
    out.write(reinterpret_cast<const char*>(&y), sizeof(y));
    out.write(reinterpret_cast<const char*>(&z), sizeof(z));
    out.write(reinterpret_cast<const char*>(&vertexId), sizeof(vertexId));
  }
  ignmsg << "File saved to: " << this->lutPath << std::endl;
}
