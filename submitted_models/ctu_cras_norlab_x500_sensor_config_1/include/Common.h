#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_COMMON_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_COMMON_HH_

/* includes() //{ */

#include <Eigen/Geometry>
#include <vector>

#include <sdf/sdf.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>

#include <Parameters.h>

//}

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

struct EigenTwist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct FrameData
{
  Eigen::Isometry3d pose;
  Eigen::Vector3d   linearVelocityWorld;
  Eigen::Vector3d   angularVelocityBody;
};

RotorConfiguration loadRotorConfiguration(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf, const Model &_model, const Entity &_comLink);

std::optional<Eigen::Matrix4Xd> calculateAllocationMatrix(const RotorConfiguration &_rotorConfiguration);

void createFrameDataComponents(EntityComponentManager &_ecm, const Entity &_link);

std::optional<FrameData> getFrameData(const EntityComponentManager &_ecm, const Entity &_link, const NoiseParameters &_noise);

inline Eigen::Matrix3d skewMatrixFromVector(const Eigen::Vector3d &_vector) {
  Eigen::Matrix3d skewMatrix;
  skewMatrix << 0, -_vector.z(), _vector.y(), _vector.z(), 0, -_vector.x(), -_vector.y(), _vector.x(), 0;
  return skewMatrix;
}

inline Eigen::Vector3d vectorFromSkewMatrix(const Eigen::Matrix3d &_skewMatrix) {
  return Eigen::Vector3d(_skewMatrix(2, 1), _skewMatrix(0, 2), _skewMatrix(1, 0));
}

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
