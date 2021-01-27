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
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <utility>
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_ign/Common.hh>
#include <subt_ign/Config.hh>
#include <subt_ign/SimpleDOTParser.hh>
#include <subt_ign/VisibilityTable.hh>

#include <fcl/config.h>
#include <fcl/data_types.h>
#include <fcl/shape/geometric_shapes.h>

#include <fcl/distance.h>

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

  std::string fullPath;
  subt::FullWorldPath(this->worldName, fullPath);

  this->worldPath = fullPath + ".sdf";
  this->graphPath = fullPath + ".dot";
  this->lutPath = fullPath + ".dat"; 

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
VisibilityCost VisibilityTable::Cost(const ignition::math::Vector3d &_from,
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
    return {std::numeric_limits<double>::max(),
            {}, {}, {},
            std::numeric_limits<double>::max()};

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
void VisibilityTable::SetModelCollisionObjects(
    const std::map<std::string, std::shared_ptr<fcl::CollisionObject>> &_collObjs)
{
  this->collisionObjs= _collObjs;
}

//////////////////////////////////////////////////
const std::map<std::tuple<int32_t, int32_t, int32_t>, uint64_t>
  &VisibilityTable::Vertices() const
{
  return this->vertices;
}

//////////////////////////////////////////////////
const std::map<uint64_t, std::vector<ignition::math::Vector3d>>
  &VisibilityTable::Breadcrumbs() const
{
  return this->breadcrumbs;
}

//////////////////////////////////////////////////
uint64_t VisibilityTable::Index(const ignition::math::Vector3d &_position) const
{
  std::vector<uint64_t> result;

  for (auto const segment : this->worldSegments)
  {
    const ignition::math::AxisAlignedBox &box = segment.first;
    if (box.Contains(_position))
      result.push_back(segment.second);
  }

  if (result.empty())
  {
    return std::numeric_limits<uint64_t>::max();
  }
  else if (result.size() == 1)
  {
    return result.front();
  }
  else 
  {
    // Fall back to using FCL to find the closest mesh.
    auto box = std::make_shared<fcl::Box>(0.01, 0.01, 0.01);
    auto boxObj = std::make_shared<fcl::CollisionObject>(box,  
      fcl::Matrix3f::getIdentity(),
      fcl::Vec3f(_position.X(), _position.Y(), _position.Z()));

    uint64_t closestIdx = 0;
    float closestDist = 1e9;

    for (const auto& resIdx: result)
    {
      // Look up name corresponding to world segment index
      auto name = this->worldSegmentNames.find(resIdx);
      if (name == this->worldSegmentNames.end())
        continue;

      // Look up fcl collision object corresponding to world
      // segment via the name
      auto meshIt = this->collisionObjs.find(name->second);
      if (meshIt == this->collisionObjs.end())
        continue;

      fcl::DistanceRequest request(true, true);
      fcl::DistanceResult fclResult;
      fcl::distance(meshIt->second.get(), boxObj.get(), request, fclResult);

      if (fclResult.min_distance < closestDist)
      {
        closestDist = fclResult.min_distance;
        closestIdx = resIdx;
      }
    }

    return closestIdx;
  }
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
      this->visibilityInfo[std::make_pair(id1, id2)] = {cost, {}, {}, {}, 0};
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
    {
      relays.insert(it->second);
      this->breadcrumbs[it->second].push_back(pose);
    }
  }

  // Compute the cost of all routes without considering relays.
  this->PopulateVisibilityInfo();

  // Update the cost of all routes considering relays.
  VisibilityInfo visibilityInfoWithRelays;
  auto vertexIds = this->visibilityGraph.Vertices();
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    for (const auto to : vertexIds)
    {
      auto id2 = to.first;
      std::set<ignition::math::graph::VertexId> visited;
      this->PopulateVisibilityInfoHelper(
        relays, id1, id2, visited, visibilityInfoWithRelays);
    }
  }

  this->visibilityInfo = visibilityInfoWithRelays;

  // Uncomment for debugging.
  // this->PrintAll(this->visibilityInfo);
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

  double srcToDstCost = srcToDstCostIt->second.cost;
  double bestRouteCost = std::numeric_limits<double>::max();
  ignition::math::graph::VertexId bestRelay = math::graph::kNullId;

  if (_from != _to)
  {
    unsigned int bestRouteNumRelaysCrossed = 0u;

    for (const auto i : _relays)
    {
      if (_from == i || _to == i)
        continue;

      // The source can't reach the relay.
      auto srcToRelayIt = this->visibilityInfo.find(std::make_pair(_from, i));
      if (srcToRelayIt == this->visibilityInfo.end())
        continue;

      // The relay can't reach destination.
      auto relayToDstIt = this->visibilityInfo.find(std::make_pair(i, _to));
      if (relayToDstIt == this->visibilityInfo.end())
        continue;

      // The source can reach the relay with this cost without breadcrumbs.
      double srcToRelayCost = srcToRelayIt->second.cost;

      // The relay can reach the relay this this cost without breadcrumbs.
      double relayToDstCost = relayToDstIt->second.cost;

      // Num relays crossed from the relay i to destination.
      unsigned int relayToDstNumRelaysCrossed = 0u;

      bool visited = _visited.find(i) != _visited.end();

      // Route not cached (with breadcrumbs).
      relayToDstIt = _visibilityInfoWithRelays.find(std::make_pair(i, _to));
      if (relayToDstIt == _visibilityInfoWithRelays.end() && !visited)
      {
        // Mark this relay as visited to avoid infinite recursion.
        _visited.insert(i);

        // Evaluate the route from the relay to the destination.
        this->PopulateVisibilityInfoHelper(
          _relays, i, _to, _visited, _visibilityInfoWithRelays);
      }

      // Do we now have a route from the relay to the destination?
      relayToDstIt = _visibilityInfoWithRelays.find(std::make_pair(i, _to));
      if (relayToDstIt != _visibilityInfoWithRelays.end())
      {
        relayToDstCost = relayToDstIt->second.cost;
        relayToDstNumRelaysCrossed = relayToDstIt->second.route.size();
      }

      // The cost of a route is the cost of its biggest hop.
      double currentRouteCost = std::max(srcToRelayCost, relayToDstCost);

      // Among all the routes found going through relays,
      // select the one with lowest cost.
      if ((currentRouteCost < bestRouteCost) ||
          (currentRouteCost == bestRouteCost &&
           relayToDstNumRelaysCrossed < bestRouteNumRelaysCrossed))
      {
        bestRouteCost = currentRouteCost;
        bestRelay = i;
        bestRouteNumRelaysCrossed = relayToDstNumRelaysCrossed;
      }
    }
  }

  // Among all posible routes, select the one with lowest cost.
  if (srcToDstCost <= bestRouteCost)
  {
    _visibilityInfoWithRelays[std::make_pair(_from, _to)] =
      {srcToDstCost, {}, {}, {}, 0};
  }
  else
  {
    std::vector<ignition::math::graph::VertexId> path;
    ignition::math::graph::VertexId relay = bestRelay;
    double maxDistance = 0;
    ignition::math::Vector3d firstBreadcrumbPos =
      ignition::math::Vector3d::Zero;
    ignition::math::Vector3d lastBreadcrumbPos = ignition::math::Vector3d::Zero;

    if (relay != math::graph::kNullId)
    {
      path.push_back(relay);
      auto relayToDstIt = _visibilityInfoWithRelays.find(
        std::make_pair(relay, _to));
      if (relayToDstIt != _visibilityInfoWithRelays.end())
      {
        auto otherRelays = relayToDstIt->second.route;
        path.insert(path.end(), otherRelays.begin(), otherRelays.end());

        // Calculate the greatest distance single hop between breadcrumbs only.
        maxDistance = this->MaxDistanceSingleHop(
          _visibilityInfoWithRelays, path, _to);
      }

      // Update the first and last breadcrumb positions.
      firstBreadcrumbPos = this->breadcrumbs.at(path.front()).front();
      lastBreadcrumbPos = this->breadcrumbs.at(path.back()).front();
    }

    _visibilityInfoWithRelays[std::make_pair(_from, _to)] =
      {bestRouteCost, path, firstBreadcrumbPos, lastBreadcrumbPos, maxDistance};
  }

  // Save the reverse route.
  auto fromToEntry = _visibilityInfoWithRelays[std::make_pair(_from, _to)];
  auto toFromPath = fromToEntry.route;
  std::reverse(std::begin(toFromPath), std::end(toFromPath));

  _visibilityInfoWithRelays[std::make_pair(_to, _from)] =
    {fromToEntry.cost, toFromPath, fromToEntry.posLastBreadcrumb,
     fromToEntry.posFirstBreadcrumb, fromToEntry.greatestDistanceSingleHop};

  return true;
}

//////////////////////////////////////////////////
double VisibilityTable::MaxDistanceSingleHop(
  const VisibilityInfo &_visibilityInfoWithRelays,
  const std::vector<ignition::math::graph::VertexId> &_relaySequence,
  const ignition::math::graph::VertexId &_to) const
{
  if (_relaySequence.size() < 2)
    return 0;

  auto firstBreadcrumb = _relaySequence.at(0);
  auto secondBreadcrumb = _relaySequence.at(1);

  // We only consider the first breadcrumb stored in the tile.
  auto posFirstBreadcrumb = this->breadcrumbs.at(firstBreadcrumb).front();
  auto posSecondBreadcrumb = this->breadcrumbs.at(secondBreadcrumb).front();

  // Distance between the first and second breadcrumbs.
  double distanceFirstHop = posFirstBreadcrumb.Distance(posSecondBreadcrumb);

  // Max distance from the second breadcrumb to the destination.
  double distanceNextHops = 0;
  auto secondRelayToDstIt = _visibilityInfoWithRelays.find(
    std::make_pair(secondBreadcrumb, _to));
  if (secondRelayToDstIt != _visibilityInfoWithRelays.end())
    distanceNextHops = secondRelayToDstIt->second.greatestDistanceSingleHop;

  return std::max(distanceFirstHop, distanceNextHops);
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

    this->worldSegments.push_back( std::make_pair(bboxIt->second, from.first));
    this->worldSegmentNames[from.first] = bboxIt->first;
  }
}

//////////////////////////////////////////////////
void VisibilityTable::BuildLUT()
{
  auto zrange = this->kMaxZ - this->kMinZ;
  std::vector<std::thread> workers;

  auto zstep = std::floor(zrange/10);

  std::atomic<int> counter = 0;

  auto functor = [&](auto zstart, auto zend) {
      for(auto z = zstart; z <= zend; ++z)
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
        counter++;
      }
  };

  for (auto z = this->kMinZ; z <= this->kMaxZ; z += zstep)
  {
    ignmsg << "Spawning worker thread: " << z << "-" << z+zstep << std::endl;
    workers.push_back(std::thread(functor, z, z + zstep));
  }

  while (counter <= zrange) {
    std::cout << std::fixed << std::setprecision(0)
              << std::min(100.0, 100.0 * counter / zrange) << " %\r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  };

  std::for_each(workers.begin(), workers.end(), [](std::thread &t){ t.join(); });
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

//////////////////////////////////////////////////
void VisibilityTable::PrintAll(const VisibilityInfo &_info) const
{
  // Get the list of vertices Id.
  auto vertexIds = this->visibilityGraph.Vertices();

  // Get the cost between all vertex pairs.
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    for (const auto to : vertexIds)
    {
      auto id2 = to.first;
      auto key = std::make_pair(id1, id2);
      auto infoIter = _info.find(key);
      // Found.
      if (infoIter != _info.end())
      {
        auto info = infoIter->second;
        std::cout << id1 << "->" << id2 << ":" << info.cost << " route [";

        for (const auto &bc : info.route)
          std::cout << bc << " ";
        std::cout << "] Max distance: " << info.greatestDistanceSingleHop
                  << " First bc: " << info.posFirstBreadcrumb
                  << " Last bc: " << info.posLastBreadcrumb << std::endl;
      }
    }
  }
  std::cout << "---" << std::endl;
}
