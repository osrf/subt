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
#ifndef SUBT_IGN_VISIBILITYTABLE_HH_
#define SUBT_IGN_VISIBILITYTABLE_HH_

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/graph/Vertex.hh>
#include <subt_ign/VisibilityTypes.hh>

namespace subt
{
  /// \brief This class stores the connectivity between pairs of sections of a
  /// world expressed as a graph.
  class VisibilityTable
  {
    /// \brief Min X to sample.
    public: static const int32_t kMinX = -20;

    /// \brief Max X to sample.
    public: static const int32_t kMaxX = 500;

    /// \brief Min Y to sample.
    public: static const int32_t kMinY = -300;

    /// \brief Max Y to sample.
    public: static const int32_t kMaxY = 300;

    /// \brief Min Z to sample.
    public: static const int32_t kMinZ = -50;

    /// \brief Max Z to sample.
    public: static const int32_t kMaxZ = 20;

    /// \brief Class constructor. Create the visibility table from a graph in
    /// DOT format.
    public: explicit VisibilityTable();

    /// \brief Load a look up table from a file. It will try to load a file
    /// located in the same directory as the world file, with the same world
    /// name but with extension .dat.
    /// E.g.: A world named 'tunnel_practice_1.world' will try to load
    /// 'tunnel_practice_1.dat'.
    /// \return True if the visibility look-up-table was loaded or false
    /// otherwise.
    public: bool Load(const std::string &_worldName);

    /// \brief Get the visibility cost.
    /// \param[in] _from A 3D position.
    /// \param[in] _to A 3D position.
    /// \return The visibility cost from one point to the other.
    public: double Cost(const ignition::math::Vector3d &_from,
                        const ignition::math::Vector3d &_to) const;

    /// \brief Generate a binary .dat file containing a list of sample points
    /// that are within the explorable areas of the world. Each sample point
    /// contained in the file also has the vertex Id of the connectivity graph
    /// associated to that point.
    public: void Generate();

    /// \brief Get the collection of sampled 3D points and their associated
    /// vertex id.
    /// \return the collection.
    public: const std::map<std::tuple<int32_t, int32_t, int32_t>, uint64_t>
      &Vertices() const;

    /// \brief Populate a graph from a file in DOT format.
    /// \param[in] _graphFilename The path to the file containing the graph.
    /// \return True if the graph was successfully generated or false otherwise.
    private: bool PopulateVisibilityGraph(const std::string &_graphFilename);

    /// \brief Populate the visibility information in memory.
    private: void PopulateVisibilityInfo();

    /// \brief Populate the visibility information in memory.
    /// \param[in] _relays Set of vertices containing breadcrumb robots.
    public: void PopulateVisibilityInfo(
                      const std::set<ignition::math::Vector3d> &_relayPoses);

    /// \brief Helper function for populating visibility information.
    /// This function updates all routes from a pair of nodes.
    /// \param[in] _relays The set of vertices containing breadcrumb robots.
    /// \param[in] _from Source vertex.
    /// \param[in] _to Destination vertex.
    /// \param[in, out] _visited Set of vertices visited. This is used to
    /// prevent infinite recursion.
    /// \param[in, out] _visibilityInfoWithRelays Visibility information.
    private: bool PopulateVisibilityInfoHelper(
      const std::set<ignition::math::graph::VertexId> &_relays,
      const ignition::math::graph::VertexId &_from,
      const ignition::math::graph::VertexId &_to,
      std::set<ignition::math::graph::VertexId> &_visited,
      VisibilityInfo &_visibilityInfoWithRelays);

    /// \brief Get the vertex Id associated to a position. The vertex Id
    /// represents the world section containing the position.
    /// \param[in] _position 3D cpublico staticordinate.
    /// \return The vertex Id.
    private: uint64_t Index(const ignition::math::Vector3d &_position) const;

    /// \brief Populate a vector, where each element is a pair with the
    /// bounding box of a model segment of the world and its associated vertex
    /// Id from the visibility graph.
    private: void CreateWorldSegments();

    /// \brief Create the visibility table in memory.
    private: void BuildLUT();

    /// \brief Generate the visibility LUT in disk.
    private: void WriteOutputFile();

    /// \brief The graph modeling the connectivity.
    private: VisibilityGraph visibilityGraph;

    /// \brief The connectivity information in a map format than can be queried.
    private: VisibilityInfo visibilityInfo;

    /// \brief All model segments used to create the environment. Each of these
    /// segments is associated with a vertex in a graph.
    /// Mapping between a model's bouding box and a vertex Id.
    private: std::vector<
             std::pair<ignition::math::AxisAlignedBox, uint64_t>> worldSegments;

    /// \brief A map that stores 3D points an the vertex id in which are located
    private: std::map<std::tuple<int32_t, int32_t, int32_t>, uint64_t> vertices;

    /// \brief The path where the Gazebo world is located.
    private: std::string worldPath;

    /// \brief The path where the .dot graph is located.
    private: std::string graphPath;

    /// \brief The path where the visibility LUT is located.
    private: std::string lutPath;

    /// \brief The world name.
    private: std::string worldName;
  };
}

#endif
