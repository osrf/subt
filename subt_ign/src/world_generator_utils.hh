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

using namespace ignition;

math::AxisAlignedBox transformAxisAlignedBox(const math::AxisAlignedBox &_box, const math::Pose3d &_pose);

std::string WorldTopStr(const std::string &_worldType, std::string outputFile, std::string worldName, bool gui);

#endif /*WORLD_GENERATOR_UTILS_H*/