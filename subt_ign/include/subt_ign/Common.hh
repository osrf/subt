/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef SUBT_IGN_COMMON_HH_
#define SUBT_IGN_COMMON_HH_

#include <subt_ign/CommonTypes.hh>
#include <subt_ign/VisibilityTypes.hh>

#include <ignition/math/Pose3.hh>

namespace subt
{
  /// \brief Mapping between artifact model names and types.
  const std::array<
      const std::pair<std::string, ArtifactType>, 15> kArtifactNames
      {
        {
          {"backpack",           ArtifactType::TYPE_BACKPACK},
          {"drill",              ArtifactType::TYPE_DRILL},
          {"duct",               ArtifactType::TYPE_DUCT},
          {"electrical_box",     ArtifactType::TYPE_ELECTRICAL_BOX},
          {"extinguisher",       ArtifactType::TYPE_EXTINGUISHER},
          {"phone",              ArtifactType::TYPE_PHONE},
          {"radio",              ArtifactType::TYPE_RADIO},
          {"rescue_randy",       ArtifactType::TYPE_RESCUE_RANDY},
          {"toolbox",            ArtifactType::TYPE_TOOLBOX},
          {"valve",              ArtifactType::TYPE_VALVE},
          {"vent",               ArtifactType::TYPE_VENT},
          {"gas",                ArtifactType::TYPE_GAS},
          {"helmet",             ArtifactType::TYPE_HELMET},
          {"rope",               ArtifactType::TYPE_ROPE},
          {"cube",               ArtifactType::TYPE_CUBE}
        }
      };

  /// \brief Mapping between enum types and strings.
  const std::array<
      const std::pair<ArtifactType, std::string>, 15> kArtifactTypes
      {
        {
          {ArtifactType::TYPE_BACKPACK      , "TYPE_BACKPACK"},
          {ArtifactType::TYPE_DRILL         , "TYPE_DRILL"},
          {ArtifactType::TYPE_DUCT          , "TYPE_DUCT"},
          {ArtifactType::TYPE_ELECTRICAL_BOX, "TYPE_ELECTRICAL_BOX"},
          {ArtifactType::TYPE_EXTINGUISHER  , "TYPE_EXTINGUISHER"},
          {ArtifactType::TYPE_PHONE         , "TYPE_PHONE"},
          {ArtifactType::TYPE_RADIO         , "TYPE_RADIO"},
          {ArtifactType::TYPE_RESCUE_RANDY  , "TYPE_RESCUE_RANDY"},
          {ArtifactType::TYPE_TOOLBOX       , "TYPE_TOOLBOX"},
          {ArtifactType::TYPE_VALVE         , "TYPE_VALVE"},
          {ArtifactType::TYPE_VENT          , "TYPE_VENT"},
          {ArtifactType::TYPE_GAS           , "TYPE_GAS"},
          {ArtifactType::TYPE_HELMET        , "TYPE_HELMET"},
          {ArtifactType::TYPE_ROPE          , "TYPE_ROPE"},
          {ArtifactType::TYPE_CUBE          , "TYPE_CUBE"}
        }
      };

  /// \brief Create an ArtifactType from a string.
  /// \param[in] _name The artifact in string format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  bool ArtifactFromString(const std::string &_name,
                          subt::ArtifactType &_type);

  /// \brief Create an ArtifactType from a string.
  /// \param[in] _name The artifact in string format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  bool ArtifactFromPartialString(const std::string &_name,
                                 subt::ArtifactType &_type);

  /// \brief Create an ArtifactType from an integer.
  //
  /// \param[in] _typeInt The artifact in int format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  bool ArtifactFromInt(const uint32_t &_typeInt,
                       subt::ArtifactType &_type);

  /// \brief Create a string from ArtifactType.
  //
  /// \param[in] _type The artifact type.
  /// \param[out] _strType The artifact string.
  /// \return True when the conversion succeed or false otherwise.
  bool StringFromArtifact(const subt::ArtifactType &_type,
                          std::string &_strType);

  /// \brief Retrieve a fully-qualifed path based on name 
  /// 
  /// This accounts for the SubT world file directory 
  /// structure.
  //
  /// \param[in] _worldName The name of the world
  ///    For example: cave_circuit_practice_02
  /// \param[out] _worldPath The world path without extension.
  /// \return True when the path is expanded successfully.
  bool FullWorldPath(const std::string &_worldName,
                     std::string &_worldPath);

  /// \brief Structure to represent an artifact as parsed
  struct Artifact
  {
    /// \brief Artifact name
    std::string name;

    /// \brief Artifact type
    subt::ArtifactType type;

    /// \brief Artifact type as a string
    std::string typeStr;

    /// \brief Artifact pose
    ignition::math::Pose3d pose;

    /// \brief Return a string representation of the artifact
    std::string String() const
    {
      std::stringstream ss;
      ss << "<Artifact:"
         << " name=" << this->name
         << " type=" << this->typeStr
         << " pose=(" << this->pose << ")"
         << ">";
      return ss.str();
    }
  };
}
#endif
