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
#include <map>
#include <string>
#include <utility>
#include <ignition/math/graph/Graph.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>
#include "subt_gazebo/SimpleDOTParser.hh"

using namespace subt;

// This is the lookup table where we store the visibility cost from any pair
// of vertices. The key is a pair containing the IDs of the two vertices.
// The value is the visibility cost between the two vertices.
using VisibilityTable =
  std::map<std::pair<ignition::math::graph::VertexId,
                     ignition::math::graph::VertexId>,
           double>;

/////////////////////////////////////////////////
/// \brief Usage.
void usage()
{
  std::cerr << "Generate a visibility lookup table.\n\n"
            << "  generate_visibility_table <graph_file> <output_file>"
            << std::endl << std::endl;
}

/////////////////////////////////////////////////
/// \brief Populate a graph from a file in DOT format.
/// \param[in] _graphFilename The path to the file containing the graph.
/// \param[out] _g The output visibility graph.
/// \return True if the graph was successfully generated or false otherwise.
bool generateVisibilyGraph(const std::string &_graphFilename,
                           VisibilityGraph &_g)
{
  std::filebuf fb;
  if (!fb.open (_graphFilename, std::ios::in))
  {
    std::cerr << "Unable to read [" << _graphFilename << "] file" << std::endl;
    return false;
  }

  std::istream is(&fb);
  SimpleDOTParser dotParser;
  return dotParser.Parse(is, _g);
}

/////////////////////////////////////////////////
/// \brief Create the visibility table in memory.
/// \param[in] _g The input visibility graph.
/// \param[out] _visibility The output table.
void buildLUT(const VisibilityGraph &_g,
              VisibilityTable &_visibility)
{
  // Get the list of vertices Id.
  auto vertexIds = _g.Vertices();

  // Get the cost between all vertex pairs.
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    auto result = ignition::math::graph::Dijkstra(_g, id1);
    for (const auto to : result)
    {
      auto id2 = to.first;
      double cost = to.second.first;
      _visibility[std::make_pair(id1, id2)] = cost;
    }
  }
}

/////////////////////////////////////////////////
/// \brief Generate the visibility LUT in disk.
/// \param[in] _outFilename The path to the output file.
/// \param[in] _g The visibility graph.
/// \param[in] _visibility The table containing all the visibility info.
/// \return True when the file was succesfully generated or false otherwise.
bool writeOutputFile(const std::string &_outFilename,
                     const VisibilityGraph &_g,
                     const VisibilityTable &_visibility)
{
  std::fstream out(_outFilename, std::ios::out | std::ios::binary);
  if (!out)
  {
    std::cerr << "Unable to create [" << _outFilename << "] file" << std::endl;
    return false;
  }

  auto vertexIds = _g.Vertices();
  auto numVertices = vertexIds.size();
  uint64_t numEntries = numVertices * numVertices;

  // First, save the number of entries.
  out.write(reinterpret_cast<const char*>(&numEntries), sizeof(numEntries));

  // Now save all entries.
  for (const auto from : vertexIds)
  {
    auto id1 = from.first;
    for (const auto to : vertexIds)
    {
      auto id2 = to.first;
      double cost = _visibility.at(std::make_pair(id1, id2));
      out.write(reinterpret_cast<const char*>(&cost), sizeof(cost));
    }
  }

  return true;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Sanity check: Make sure there is the right number of arguments.
  if (argc != 3)
  {
    usage();
    return -1;
  }

  // Parse the command line arguments.
  std::string graphFileName = argv[1];
  std::string outFilename = argv[2];

  // Generate the graph.
  VisibilityGraph g;
  if (!generateVisibilyGraph(graphFileName, g))
    return -1;

  // Build the LUT.
  VisibilityTable visibility;
  buildLUT(g, visibility);

  // Write the output file.
  if (!writeOutputFile(outFilename, g, visibility))
    return -1;

  return 0;
}
