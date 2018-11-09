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
double VisibilityTable::Cost(const ignition::math::Vector3d &_from,
  const ignition::math::Vector3d &_to) const
{
  uint64_t from = this->Index(_from);
  uint64_t to = this->Index(_to);

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
  // offset_x = (x - min_x) / step_x
  auto offsetX = (_position.X() - this->attributesX[0]) / this->attributesX[2];

  // offset_y = (y - min_y) / step_y
  auto offsetY = (_position.Y() - this->attributesY[0]) / this->attributesY[2];

  // offset_z = (z - min_z) / step_z
  auto offsetZ = (_position.Z() - this->attributesZ[0]) / this->attributesZ[2];

  return offsetZ * this->levelSize + offsetY * this->rowSize + offsetX;
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

  return this->PopulateAttributes(dotParser);
}

//////////////////////////////////////////////////
bool VisibilityTable::PopulateAttributes(const SimpleDOTParser &_parser)
{
  // Sanity check: Make sure that all required attributes are found.
  auto hiddenAttributes = _parser.HiddenAttributes();
  const auto kRequiredAttributes = {
    "min_x", "max_x", "step_x",
    "min_y", "max_y", "step_y",
    "min_z", "max_z", "step_z"
  };
  for (auto const attr : kRequiredAttributes)
  {
    if (hiddenAttributes.find(attr) == hiddenAttributes.end())
    {
      std::cerr << "Visibility table error: Attribute [" << attr << "] not "
                << "found" << std::endl;
      return false;
    }
  }

  std::array<std::string, 3> coords = {"x", "y", "z"};
  std::map<std::string, std::array<double, 3>> attributes;

  for (auto coord : coords)
  {
    std::array<std::string, 3> attributesStr =
      {
        hiddenAttributes["min_"  + coord],
        hiddenAttributes["max_"  + coord],
        hiddenAttributes["step_" + coord]
      };

    std::array<double, 3> attributesDouble;
    for (auto i = 0; i < 3; ++i)
    {
      std::string::size_type sz;
      try
      {
        attributesDouble[i] = std::stod(attributesStr[i], &sz);
      }
      catch(...)
      {
        std::cerr << "Visibility table error: Error parsing ["
                  << attributesStr[i] << "] attribute value as a double"
                  << std::endl;
        return false;
      }
    }

    attributes[coord] = attributesDouble;
  }

  this->attributesX = attributes["x"];
  this->attributesY = attributes["y"];
  this->attributesZ = attributes["z"];

  // range_x = max_x - min_x
  auto rangeX = this->attributesX[1] - this->attributesX[0];
  this->rowSize = rangeX / this->attributesX[2];

  // range_y = max_y - min_y
  auto rangeY = this->attributesY[1] - this->attributesY[0];
  uint64_t colSize = rangeY / this->attributesY[2];

  this->levelSize = colSize * rowSize;

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
