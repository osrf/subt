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
#ifndef SUBT_IGN_VISIBILITYTYPES_HH_
#define SUBT_IGN_VISIBILITYTYPES_HH_

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <ignition/math/graph/Graph.hh>
#include <ignition/math/Vector3.hh>

namespace subt
{

/// \brief A class that contains some information about the cost of
/// communicating two robots.
class VisibilityCost
{
  /// \brief The cost from one tile to another tile.
  public: double cost;

  /// \brief The best route connecting source and destination. No that route
  /// only contains the sequence of breadcrumbs.
  public: std::vector<ignition::math::graph::VertexId> route;

  /// \brief The position of the first breadcrumb in the route. This value
  /// should be ignored if route is empty (no breadcrumbs).
  public: ignition::math::Vector3d posFirstBreadcrumb;

  /// \brief The position of the last breadcrumb in the route. This value
  /// should be ignored if route is empty (no breadcrumbs).
  public: ignition::math::Vector3d posLastBreadcrumb;

  /// \brief A path from source to destination can traverse 0-N breadcrumbs.
  /// This field stores the maximum distance between a pair of breadcrumbs.
  /// E.g.: source->A->B->C->destination. Let's assume the next
  /// Euclidean distances: dist(A, B): 2; dist(B, C): 3. This field stores a 3.
  public: double greatestDistanceSingleHop;
};

/// \def VisibilityGraph
/// \brief An undirected graph to represent communication visibility between
/// different areas of the world.
using VisibilityGraph =
    ignition::math::graph::UndirectedGraph<std::string, uint8_t>;

// This is the lookup table where we store the visibility cost from any pair
// of vertices. The key is a pair containing the IDs of the two vertices.
// The value is the visibility cost between the two vertices.
using VisibilityInfo =
    std::map<std::pair<ignition::math::graph::VertexId,
                       ignition::math::graph::VertexId>,
             VisibilityCost>;
}
#endif
