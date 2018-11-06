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
#include <ignition/math/graph/GraphAlgorithms.hh>
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
double VisibilityTable::Cost(const ignition::math::Vector3d &/*_from*/,
  const ignition::math::Vector3d &/*_to*/) const
{
  return 0.0;
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
  bool result = dotParser.Parse(is);

  this->visibilityGraph = dotParser.Graph();
  // this->range = dotParser.Range();
  // this->stepSize = dotParser.StepSize();

  return result;
}

//////////////////////////////////////////////////
void VisibilityTable::PopulateVisibilityInfo()
{
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
}
