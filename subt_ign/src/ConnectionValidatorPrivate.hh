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

#ifndef SUBT_IGN_CONNECTIONVALIDATORPRIVATE_HH_
#define SUBT_IGN_CONNECTIONVALIDATORPRIVATE_HH_

#include <string>

#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport/Node.hh>

#include <subt_ign/SimpleDOTParser.hh>
#include "ConnectionHelper.hh"

namespace subt
{
  /// \brief Data about connections between tiles in the environment
  struct Connection
  {
    /// \brief ID of the connection (from dot graph)
    int id;

    /// \brief Pointer to first
    VertexData *tile1;

    /// \brief Pointer to second tile
    VertexData *tile2;

    /// \brief Computed connection point between the tiles.
    ignition::math::Vector3d connectionPoint;
  };

  /// \brief Private data for the connection validator
  class ConnectionValidatorPrivate
  {
    /// \brief Load both the SDF and DOT graph content for a given world
    /// \param[in] _worldName name of the world to load (eg "urban_qual")
    /// \return True when all content is succesfully loaded, false otherwise.
    public: bool Load(const std::string &_worldName);

    /// \brief Load SDF content for a given world
    /// \param[in] _worldName name of the world to load (eg "urban_qual")
    /// \return True when all content is succesfully loaded, false otherwise.
    private: bool LoadSdf(const std::string &_fname);

    /// \brief Load connectivity graph content for a given world
    /// \param[in] _worldName name of the world to load (eg "urban_qual")
    /// \return True when all content is succesfully loaded, false otherwise.
    private: bool LoadDot(const std::string &_fname);

    /// \brief Extract connection data from the graph/sdf world.
    public: void PopulateConnections();

    /// \brief Move the validator actor to a given point
    /// \param[in] _pt Point to move artifact to
    /// \return True when service call was successful, false otherwise.
    public: bool Move(const ignition::math::Vector3d& _pt);

    /// \brief Callback fired when the next service is called.
    /// \param[in] _msg Service request
    /// \param[out] _rep Service response
    public: bool OnNext(const ignition::msgs::StringMsg &_msg,
                        ignition::msgs::StringMsg &_rep);

    /// \brief Callback fired when the next service is called.
    /// \param[in] _msg Service request
    /// \param[out] _rep Service response
    public: bool OnPrev(const ignition::msgs::StringMsg &_msg,
                        ignition::msgs::StringMsg &_rep);

    /// \brief Name of the world.
    public: std::string worldName;

    /// \brief SDF content of the world.
    public: sdf::Root sdfRoot;

    /// \brief Visibility graph content of the world.
    public: VisibilityGraph graph;

    /// \brief Ignition transport node.
    public: ignition::transport::Node node;

    /// \brief Vertex data from SDF/dot, key is tile name (eg "tile_1")
    public: std::map<std::string, VertexData> vertData;

    /// \brief Connection data from SDF/dot
    public: std::vector<Connection> connData;

    /// \brief Iterator to current location in the connection data.
    /// Used in support of next/prev service calls
    public: std::vector<Connection>::iterator connDataIter;
  };
}

#endif  // SUBT_IGN_CONNECTIONVALIDATORPRIVATE_HH_

