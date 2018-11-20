/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <fstream>
#include <iostream>
#include <utility>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>
#include <ignition/common/Util.hh>

#include "subt_gazebo/SimpleDOTParser.hh"
#include "subt_gazebo/VisibilityTable.hh"

using namespace subt;

//////////////////////////////////////////////////
VisibilityTable::VisibilityTable(const std::string &_worldName)
{
  this->worldName = _worldName();
  // std::string filePath = ignition::common::joinPaths(
  //   SUBT_GAZEBO_PROJECT_SOURCE_PATH, "worlds", worldName + ".dot");
  // std::cout << "File path: [" << filePath << "]" << std::endl;


  std::cout << "Populating graph" << std::endl;
  if (!this->PopulateVisibilityGraph(_graphFilename))
    return;

  std::cout << "Populating visibilityInfo" << std::endl;
  this->PopulateVisibilityInfo();

  std::cout << "Creating LUT" << std::endl;
  this->CreateVertexIdsLUT();
  std::cout << "LUT created" << std::endl;
};

//////////////////////////////////////////////////
double VisibilityTable::Cost(const ignition::math::Vector3d &_from,
  const ignition::math::Vector3d &_to) const
{
  // std::cout << "Cost from [" << _from << "] to [" << _to << "]" << std::endl;

  uint32_t x = std::round(_from.X());
  uint32_t y = std::round(_from.Y());
  uint32_t z = std::round(_from.Z());
  auto sampleFrom = std::make_tuple(x, y, z);

  x = std::round(_to.X());
  y = std::round(_to.Y());
  z = std::round(_to.Z());
  auto sampleTo = std::make_tuple(x, y, z);

  uint64_t from = std::numeric_limits<uint64_t>::max();
  if (this->vertices.find(sampleFrom) != this->vertices.end())
    from = this->vertices.at(sampleFrom);

  uint64_t to = std::numeric_limits<uint64_t>::max();
  if (this->vertices.find(sampleTo) != this->vertices.end())
    to = this->vertices.at(sampleTo);
  
  // std::cout << "Vertex from: " << from << std::endl;
  // std::cout << "Vertex from (Index): " << this->Index(_from) << std::endl;
  // std::cout << "Vertex to: " << to << std::endl;
  // std::cout << "Vertex to (Index): " << this->Index(_to) << std::endl;

  auto key = std::make_pair(from, to);
  if (this->visibilityInfo.find(key) == this->visibilityInfo.end())
  {
    // std::cerr << "VisibilityTable::Cost error: Unable to find (" << from
    //           << "," << to << ") index from [" << _from << "] and ["
    //           << _to << "]" << std::endl;
    return -1;
  }

  return this->visibilityInfo.at(key);
}

//////////////////////////////////////////////////
uint64_t VisibilityTable::Index(const ignition::math::Vector3d &_position) const
{
  for (auto const segment : this->worldSegments)
  {
    auto model = segment.first;
    auto modelPose = model->WorldPose();
    auto boundingBox = model->BoundingBox();
    if (boundingBox.Contains(_position))
    {
      //std::cout << "Segment found inside bounding box " << boundingBox << std::endl;
      //std::cout << "Model: " << modelPose.Pos() << std::endl;
      return segment.second;
    }
  }

  return std::numeric_limits<uint64_t>::max();
}

//////////////////////////////////////////////////
void VisibilityTable::CreateVertexIdsLUT()
{
  for (auto z = -30; z <= 20; z += 1)
  {
    for (auto y = -100; y <= 100; y += 1)
      for (auto x = -20; x <= 500; x += 1)
      {
        ignition::math::Vector3d position(x, y, z);
        auto index = this->Index(position);
        if (index != std::numeric_limits<uint64_t>::max())
          this->vertices[std::make_tuple(x, y, z)] = index;
      }
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
  // Get the list of vertices Id.
  auto vertexIds = this->visibilityGraph.Vertices();

  // Get the cost between all vertex pairs.
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
    auto model = gazebo::physics::get_world()->ModelByName(modelName);
    if (!model)
    {
      std::cerr << "Unable to find model [" << modelName << "]. "
                << "Ignoring vertex" << std::endl;
      continue;
    }
    
    // Add the bounding box and Id pair to the list.
    this->worldSegments.push_back(std::make_pair(model, from.first));
  
    auto id1 = from.first;
    auto result = ignition::math::graph::Dijkstra(this->visibilityGraph, id1);
    for (const auto to : result)
    {
      auto id2 = to.first;
      double cost = to.second.first;
      this->visibilityInfo[std::make_pair(id1, id2)] = cost;
    }
  }
}

//////////////////////////////////////////////////
void VisibilityTable::Generate()
{

}
