/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef SUBT_IGN_CONNECTIONHELPER_HH_
#define SUBT_IGN_CONNECTIONHELPER_HH_

#include <string>
#include <map>
#include <vector>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

namespace subt
{
  /// \brief Data about vertex (tile) in the environment
  struct VertexData
  {
    /// \brief ID of the vertex (from dot graph)
    int id;

    /// \brief Type of the tile (eg "Urban Straight")
    std::string tileType;

    /// \brief Name of the tile as it appears in the SDF file. (eg "tile_1")
    std::string tileName;

    /// \brief SDF model content for the tile
    sdf::Model model;
  };

  /// \brief Helper class for creating and validating connection points
  class ConnectionHelper
  {
    /// \brief Enum of connection type
    public: enum ConnectionType
            {
              /// \brief Straight tile
              STRAIGHT = 0,
              /// \brief Turn tile (includes corners, 3 way, and intersections)
              TURN = 1,
            };

    /// \brief Enum of circuit type
    public: enum CircuitType
            {
              /// \brief Tunnel tiles
              TUNNEL       = 0,
              /// \brief Urban tiles
              URBAN        = 1,
              /// \brief Cave tiles
              CAVE         = 2,
              /// \brief Universal tiles
              UNIVERSAL    = 3,
              /// \brief Transition tiles
              TRANSITION   = 4,
              /// \brief Staging are
              STAGING_AREA = 5,
            };

    /// \brief Compute the connection point between two tiles.
    /// This uses a reference list of connection points for each tile type.
    /// The function iterates over the connection points transformed into the
    /// world coordinate frame to find the location where the tiles meet.
    /// \param[in] _tile1 vertex data for first connected tile
    /// \param[in] _tile2 vertex data for second connected tile
    /// \param[out] _pt Populated point data
    /// \return True if connection point was found, false otherwise.
    public: static bool ComputePoint(VertexData *_tile1, VertexData *_tile2,
                      ignition::math::Vector3d& _pt);

    public: static std::vector<ignition::math::Vector3d> GetConnectionPoints(VertexData *_tile1);

    /// \brief Map of tile type to a vector of connection points
    public: static std::map<std::string, std::vector<ignition::math::Vector3d>>
                      connectionPoints;

    /// \brief Map of tile type to connection type
    public: static std::map<std::string, subt::ConnectionHelper::ConnectionType>
                      connectionTypes;

    /// \brief Map of tile type to circuit type
    public: static std::map<std::string, subt::ConnectionHelper::CircuitType>
                      circuitTypes;
  };
}
#endif
