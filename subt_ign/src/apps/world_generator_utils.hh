#ifndef WORLD_GENERATOR_UTILS_H
#define WORLD_GENERATOR_UTILS_H

#include <list>
#include <string>
#include <sstream>
#include <unistd.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/ClientConfig.hh>

#include "ConnectionHelper.hh"
#include "world_generator_base.hh"

using namespace ignition;

math::AxisAlignedBox transformAxisAlignedBox(const math::AxisAlignedBox &_box, const math::Pose3d &_pose);

/// \brief Global helper function to create a world section from an individual file
/// \param[in] _type Type of tile
/// \param[in] _entry Entry position of this tile. This should be one of the
/// connection points of the tile
/// \param[in] _rot Rotation to be applied to tile so that its entry point is
/// in +X direction
/// \param[in] _tileType Type of tile
WorldSection CreateWorldSectionFromTile(const std::string &_type,
        const math::Vector3d &_entry, const math::Quaterniond &_rot,
        TileType _tileType);

/// \brief Global helper function to return top sdf file string
/// \param[in] _worldType Type of sub-domain
/// \param[in] outputFile Filename for world model
/// \param[in] worldName Name of the world created
/// \param[in] gui Bool for whether gui string is included or not 
std::string WorldTopStr(const std::string &_worldType, std::string outputFile, std::string worldName, bool gui);

/// \brief Global function helper to return the proper orientation of certain tiles
/// \param[in] tileName Name of the tile whose rotation must be returned
void TunnelTileRotations(const std::string &_tileName, math::Vector3d &_pt, math::Quaterniond &_rot);

#endif /*WORLD_GENERATOR_UTILS_H*/