#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_SE3CONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTER_CONTROL_SE3CONTROLLER_HH_

#include <Eigen/Geometry>
#include <Common.h>
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

struct SE3ControllerParameters
{
  Eigen::Vector3d velocity_gain;
  Eigen::Vector3d attitude_gain;
  Eigen::Vector3d angular_rate_gain;
  Eigen::Vector3d max_linear_acceleration;
};

struct SE3ControllerFeedforward
{
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
};

class SE3Controller {

public:
  SE3Controller(const SE3ControllerParameters &controller_parameters, const VehicleParameters &vehicle_parameters);

  // the main function that returns the result
  Eigen::VectorXd CalculateRotorVelocities(const FrameData &simulator_model_data, const EigenTwist &control_command,
                                           const SE3ControllerFeedforward &feedforward_command) const;

private:
  // | ----------------------- parameters ----------------------- |

  SE3ControllerParameters _controller_parameters_;
  VehicleParameters       _vehicle_parameters_;

  bool InitializeParameters();

  // parameters computed in during initialization
  Eigen::Vector3d  normalized_attitude_gain_;
  Eigen::Vector3d  normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  // | -------------------- internal methods -------------------- |

  Eigen::Vector3d ComputeDesiredAcceleration(const FrameData &simulator_model_data, const Eigen::Vector3d &vel_ref, const Eigen::Vector3d &acc_ref) const;
  Eigen::Vector3d SO3Controller(const FrameData &simulator_model_data, const Eigen::Vector3d &des_acceleration, const Eigen::Vector3d &des_jerk,
                                const double &des_yaw_rate) const;
};

}  // namespace multicopter_control

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
