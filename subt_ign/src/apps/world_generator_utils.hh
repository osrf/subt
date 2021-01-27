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


std::string WorldTopStr(const std::string &_worldType, std::string outputFile, std::string worldName, bool gui);

#endif /*WORLD_GENERATOR_UTILS_H*/