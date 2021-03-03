/* includes //{ */

#include <BacaSE3Controller.hh>

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

// | ------------------------- PUBLIC ------------------------- |

/* BacaSE3Controller() //{ */

BacaSE3Controller::BacaSE3Controller(const BacaSE3ControllerParameters &controller_parameters, const VehicleParameters &vehicle_parameters) {

  _controller_parameters_ = controller_parameters;
  _vehicle_parameters_    = vehicle_parameters;

  InitializeParameters();
}

//}

/* CalculateRotorVelocities() //{ */

Eigen::VectorXd BacaSE3Controller::CalculateRotorVelocities(const FrameData &simulator_model_data, const EigenTwist &control_command) const {

  Eigen::Vector3d des_acceleration = this->ComputeDesiredAcceleration(simulator_model_data, control_command);

  Eigen::Vector3d des_angular_acceleration = this->SO3Controller(simulator_model_data, control_command, des_acceleration);

  // Project thrust onto body z axis.
  double thrust = -_vehicle_parameters_.mass * des_acceleration.dot(simulator_model_data.pose.linear().col(2));

  Eigen::Vector4d angularAccelerationThrust;

  angularAccelerationThrust.block<3, 1>(0, 0) = des_angular_acceleration;
  angularAccelerationThrust(3)                = thrust;

  Eigen::VectorXd rotor_velocities = angular_acc_to_rotor_velocities_ * angularAccelerationThrust;

  rotor_velocities = rotor_velocities.cwiseMax(Eigen::VectorXd::Zero(rotor_velocities.rows()));
  rotor_velocities = rotor_velocities.cwiseSqrt();

  return rotor_velocities;
}

//}

// | ------------------------- PRIVATE ------------------------ |

/* InitializeParameters() //{ */

bool BacaSE3Controller::InitializeParameters() {

  auto allocationMatrix = calculateAllocationMatrix(_vehicle_parameters_.rotorConfiguration);

  if (!allocationMatrix.has_value()) {
    // Error should already be printed by function
    return false;
  }

  // make inertia-independent attitude gain
  normalized_attitude_gain_ = _controller_parameters_.attitudeGain.transpose() * _vehicle_parameters_.inertia.inverse();

  // make inertia-independent attitude rate gain
  normalized_angular_rate_gain_ = _controller_parameters_.angularRateGain.transpose() * _vehicle_parameters_.inertia.inverse();

  {  // TODO check what this really does and where it comes from

    Eigen::Matrix4d moi;
    moi.setZero();
    moi.block<3, 3>(0, 0) = _vehicle_parameters_.inertia;
    moi(3, 3)             = 1;

    angular_acc_to_rotor_velocities_.resize(_vehicle_parameters_.rotorConfiguration.size(), 4);

    const auto &aMat = *allocationMatrix;

    // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia
    // matrix I. A^{ \dagger} = A^T*(A*A^T)^{-1}
    angular_acc_to_rotor_velocities_ = aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;
  }

  return true;
}

//}

/* ComputeDesiredAcceleration() //{ */

Eigen::Vector3d BacaSE3Controller::ComputeDesiredAcceleration(const FrameData &simulator_model_data, const EigenTwist &control_command) const {

  Eigen::Vector3d velocityError = simulator_model_data.linearVelocityWorld - simulator_model_data.pose.linear() * control_command.linear;

  Eigen::Vector3d accelCommand = velocityError.cwiseProduct(_controller_parameters_.velocityGain) / _vehicle_parameters_.mass;

  accelCommand = accelCommand.cwiseAbs().cwiseMin(_controller_parameters_.maxLinearAcceleration).cwiseProduct(accelCommand.cwiseSign());

  return accelCommand + _vehicle_parameters_.gravity;
}

//}

/* SO3Controller() //{ */

Eigen::Vector3d BacaSE3Controller::SO3Controller(const FrameData &simulator_model_data, const EigenTwist &control_command,
                                                 const Eigen::Vector3d &_acceleration) const {

  const Eigen::Matrix3d &rot = simulator_model_data.pose.linear();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1Des = rot.col(0);

  Eigen::Vector3d b3Des;
  b3Des = -_acceleration / _acceleration.norm();

  // Check if b1 and b3 are parallel. If so, choose a different b1 vector. This
  // could happen if the UAV is rotated by 90 degrees w.r.t the horizontal
  // plane.
  const double tol = 1e-3;
  if (b1Des.cross(b3Des).squaredNorm() < tol) {
    // acceleration and b1 are parallel. Choose a different vector
    b1Des = rot.col(1);

    if (b1Des.cross(b3Des).squaredNorm() < tol) {
      b1Des = rot.col(2);
    }
  }

  Eigen::Vector3d b2Des;
  b2Des = b3Des.cross(b1Des);
  b2Des.normalize();

  Eigen::Matrix3d rotDes;
  rotDes.col(0) = b2Des.cross(b3Des);
  rotDes.col(1) = b2Des;
  rotDes.col(2) = b3Des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix = 0.5 * (rotDes.transpose() * rot - rot.transpose() * rotDes);
  Eigen::Vector3d angleError       = vectorFromSkewMatrix(angleErrorMatrix);

  // | ------------------ angular rate control ------------------ |

  Eigen::Vector3d des_angular_rate(Eigen::Vector3d::Zero());

  des_angular_rate[2] = control_command.angular[2];  // set the 'yaw rate' from the control command

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = simulator_model_data.angularVelocityBody - rot.transpose() * rotDes * des_angular_rate;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = _vehicle_parameters_.inertia;
  // const Eigen::Vector3d &omega = simulator_model_data.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            rot.transpose() * rotDes * des_angular_rate);

  // std::cout << moiTerm2.transpose() << std::endl;
  // return -1 * angleError.cwiseProduct(normalized_attitude_gain_) -
  //         angularRateError.cwiseProduct(normalized_angular_rate_gain_) +
  //         moiTerm - moiTerm2;
  return -1 * angleError.cwiseProduct(normalized_attitude_gain_) - angularRateError.cwiseProduct(normalized_angular_rate_gain_);
}

//}

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
