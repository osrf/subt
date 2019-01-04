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
#ifndef SUBT_GAZEBO_COMMONTYPES_HH_
#define SUBT_GAZEBO_COMMONTYPES_HH_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math/graph/Graph.hh>

namespace subt
{
  /// \def Neighbors_M
  /// \brief Map of neighbors
  using Neighbors_M = std::map<std::string, double>;

  /// \brief All the supported artifact types.
  enum class ArtifactType : uint32_t
  {
    TYPE_BACKPACK       = 0,
    TYPE_DUCT           = 1,
    TYPE_ELECTRICAL_BOX = 2,
    TYPE_EXTINGUISHER   = 3,
    TYPE_PHONE          = 4,
    TYPE_RADIO          = 5,
    TYPE_TOOLBOX        = 6,
    TYPE_VALVE          = 7,
  };

  /// \brief The name of the base station in Gazebo. This is used as the
  /// address to report artifacts.
  const std::string kBaseStationName = "BaseStation";

  /// \brief Internal service used to report new artifacts.
  const std::string kNewArtifactSrv = "/subt/artifacts/new";

  /// \brief Service used to visualize network connectivity.
  const std::string kVisualizeCommsModelSrv = "/subt/comms_model/visualize";

  /// \brief Scoped name of collision which detects an entering object in the
  /// start area to initiate the game.
  const std::string kStartCollisionName  = "start_area::link::collision";
}
#endif
