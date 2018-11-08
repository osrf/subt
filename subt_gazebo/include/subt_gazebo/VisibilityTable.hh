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

#include <array>
#include <string>
#include <ignition/math/Vector3.hh>
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/SimpleDOTParser.hh"

#ifndef SUBT_GAZEBO_VISIBILITYTABLE_HH_
#define SUBT_GAZEBO_VISIBILITYTABLE_HH_

namespace subt
{
  /// \brief This class stores the connectivity between pairs of sections of a
  /// world expressed as a graph.
  class VisibilityTable
  {
    /// \brief Class constructor. Create the visibility table from a graph in
    /// DOT format.
    /// \param[in] _graphFilename The path to the file containing the graph.
    public: explicit VisibilityTable(const std::string &_graphFilename);

    /// \brief Get the visibility cost.
    /// \param[in] _from A 3D position.
    /// \param[in] _to A 3D position.
    /// \return The visibility cost from one point to the other.
    public: double Cost(const ignition::math::Vector3d &_from,
                        const ignition::math::Vector3d &_to) const;

    /// \brief Populate a graph from a file in DOT format.
    /// \param[in] _graphFilename The path to the file containing the graph.
    /// \return True if the graph was successfully generated or false otherwise.
    private: bool PopulateVisibilityGraph(const std::string &_graphFilename);

    /// \brief ToDo.
    private: bool PopulateAttributes(const SimpleDOTParser &_parser);

    /// \brief Populate the visibility information in memory.
    private: void PopulateVisibilityInfo();

    /// \brief Generate an index from a 3D coordinate.
    /// \param[in] _position 3D coordinate.
    /// \return The index.
    private: uint64_t Index(const ignition::math::Vector3d &_position) const;

    /// \brief The graph modeling the connectivity.
    private: VisibilityGraph visibilityGraph;

    /// \brief The connectivity information in a map format than can be queried.
    private: VisibilityInfo visibilityInfo;

    /// \brief The X attributes [min, max, step].
    private: std::array<double, 3> attributesX;

    /// \brief The Y attributes [min, max, step].
    private: std::array<double, 3> attributesY;

    /// \brief The Z attributes [min, max, step].
    private: std::array<double, 3> attributesZ;
  };
}

#endif
