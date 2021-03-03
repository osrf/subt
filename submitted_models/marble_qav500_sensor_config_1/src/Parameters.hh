#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROLLER_PARAMETERS_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROLLER_PARAMETERS_HH_

#include <Eigen/Geometry>
#include <vector>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
namespace multicopter_control
{
struct Rotor
{
  double angle;
  double armLength;
  double forceConstant;
  double momentConstant;
  int    direction;
};

using RotorConfiguration = std::vector<Rotor>;

struct VehicleParameters
{
  double             mass;
  Eigen::Matrix3d    inertia;
  Eigen::Vector3d    gravity;
  RotorConfiguration rotorConfiguration;
};

struct NoiseParameters
{
  Eigen::Vector3d linearVelocityMean;
  Eigen::Vector3d linearVelocityStdDev;
  Eigen::Vector3d angularVelocityMean;
  Eigen::Vector3d angularVelocityStdDev;
};

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
