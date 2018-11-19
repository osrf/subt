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
VisibilityTable::VisibilityTable(const std::string &_graphFilename)
{
  if (!this->PopulateVisibilityGraph(_graphFilename))
    return;

  this->PopulateVisibilityInfo();
};

//////////////////////////////////////////////////
double VisibilityTable::Cost(const ignition::math::Vector3d &_from,
  const ignition::math::Vector3d &_to) const
{
  std::cout << "Cost from [" << _from << "] to [" << _to << "]" << std::endl;
  uint64_t from = this->Index(_from);
  uint64_t to = this->Index(_to);
  std::cout << "Vertex from: " << from << std::endl;
  std::cout << "Vertex to: " << to << std::endl;

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
      std::cout << "Segment found inside bounding box " << boundingBox << std::endl;
      std::cout << "Model: " << modelPose.Pos() << std::endl;
      return segment.second;
    }
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
