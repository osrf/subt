#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_BACASE3CONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_BACASE3CONTROLLER_HH_

#include <Eigen/Geometry>
#include <Common.hh>
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

struct BacaSE3ControllerParameters
{
  Eigen::Vector3d velocityGain;
  Eigen::Vector3d attitudeGain;
  Eigen::Vector3d angularRateGain;
  Eigen::Vector3d maxLinearAcceleration;
};

class BacaSE3Controller {

public:
  BacaSE3Controller(const BacaSE3ControllerParameters &controller_parameters, const VehicleParameters &vehicle_parameters);

  // the main function that returns the result
  Eigen::VectorXd CalculateRotorVelocities(const FrameData &simulator_model_data, const EigenTwist &control_command) const;

private:
  // | ----------------------- parameters ----------------------- |

  BacaSE3ControllerParameters _controller_parameters_;
  VehicleParameters           _vehicle_parameters_;

  bool InitializeParameters();

  // parameters computed in during initialization
  Eigen::Vector3d  normalized_attitude_gain_;
  Eigen::Vector3d  normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  // | -------------------- internal methods -------------------- |

  Eigen::Vector3d ComputeDesiredAcceleration(const FrameData &simulator_model_data, const EigenTwist &control_command) const;
  Eigen::Vector3d SO3Controller(const FrameData &simulator_model_data, const EigenTwist &control_command, const Eigen::Vector3d &_acceleration) const;
};

}  // namespace multicopter_control

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
