#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_LEEVELOCITYCONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_LEEVELOCITYCONTROLLER_HH_

#include <Eigen/Geometry>
#include <memory>
#include "ignition/gazebo/config.hh"

#include "Common.hh"

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
struct LeeVelocityControllerParameters
{
  Eigen::Vector3d velocityGain;
  Eigen::Vector3d attitudeGain;
  Eigen::Vector3d angularRateGain;
  Eigen::Vector3d maxLinearAcceleration;
};

class LeeVelocityController {

public:
  static std::unique_ptr<LeeVelocityController> MakeController(const LeeVelocityControllerParameters &_controllerParams,
                                                               const VehicleParameters &              _vehicleParams);

  void CalculateRotorVelocities(const FrameData &_frameData, const EigenTwist &_cmdVel, Eigen::VectorXd &_rotorVelocities) const;

private:
  LeeVelocityController() = default;

  Eigen::Vector3d ComputeDesiredAcceleration(const FrameData &_frameData, const EigenTwist &_cmdVel) const;

  Eigen::Vector3d ComputeDesiredAngularAcc(const FrameData &_frameData, const EigenTwist &_cmdVel, const Eigen::Vector3d &_acceleration) const;

  bool InitializeParameters();

  LeeVelocityControllerParameters controllerParameters;

  VehicleParameters vehicleParameters;

  Eigen::Vector3d normalizedAttitudeGain;

  Eigen::Vector3d normalizedAngularRateGain;

  Eigen::MatrixX4d angularAccToRotorVelocities;
};

}  // namespace multicopter_control

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
