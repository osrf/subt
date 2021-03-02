#include "LeeVelocityController.hh"

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

/* MakeController() //{ */

std::unique_ptr<LeeVelocityController> LeeVelocityController::MakeController(const LeeVelocityControllerParameters &_controllerParams,
                                                                             const VehicleParameters &              _vehicleParams) {

  std::unique_ptr<LeeVelocityController> controller(new LeeVelocityController());

  controller->controllerParameters = _controllerParams;
  controller->vehicleParameters    = _vehicleParams;

  if (controller->InitializeParameters()) {
    return controller;
  } else {
    return nullptr;
  }
}

//}

/* InitializeParameters() //{ */

bool LeeVelocityController::InitializeParameters() {

  auto allocationMatrix = calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);

  if (!allocationMatrix.has_value()) {
    // Error should already be printed by function
    return false;
  }

  // To make the tuning independent of the inertia matrix we divide here.
  this->normalizedAttitudeGain = this->controllerParameters.attitudeGain.transpose() * this->vehicleParameters.inertia.inverse();

  this->normalizedAngularRateGain = this->controllerParameters.angularRateGain.transpose() * this->vehicleParameters.inertia.inverse();

  Eigen::Matrix4d moi;
  moi.setZero();
  moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
  moi(3, 3)             = 1;

  this->angularAccToRotorVelocities.resize(this->vehicleParameters.rotorConfiguration.size(), 4);

  const auto &aMat = *allocationMatrix;

  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia
  // matrix I. A^{ \dagger} = A^T*(A*A^T)^{-1}
  this->angularAccToRotorVelocities = aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

  return true;
}

//}

/* CalculateRotorVelocities() //{ */

void LeeVelocityController::CalculateRotorVelocities(const FrameData &_frameData, const EigenTwist &_cmdVel, Eigen::VectorXd &_rotorVelocities) const {

  Eigen::Vector3d acceleration = this->ComputeDesiredAcceleration(_frameData, _cmdVel);

  Eigen::Vector3d angularAcceleration = this->ComputeDesiredAngularAcc(_frameData, _cmdVel, acceleration);

  // Project thrust onto body z axis.
  double thrust = -this->vehicleParameters.mass * acceleration.dot(_frameData.pose.linear().col(2));

  Eigen::Vector4d angularAccelerationThrust;

  angularAccelerationThrust.block<3, 1>(0, 0) = angularAcceleration;
  angularAccelerationThrust(3)                = thrust;

  _rotorVelocities = this->angularAccToRotorVelocities * angularAccelerationThrust;

  _rotorVelocities = _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
  _rotorVelocities = _rotorVelocities.cwiseSqrt();
}

//}

/* ComputeDesiredAcceleration() //{ */

Eigen::Vector3d LeeVelocityController::ComputeDesiredAcceleration(const FrameData &_frameData, const EigenTwist &_cmdVel) const {

  Eigen::Vector3d velocityError = _frameData.linearVelocityWorld - _frameData.pose.linear() * _cmdVel.linear;

  Eigen::Vector3d accelCommand = velocityError.cwiseProduct(this->controllerParameters.velocityGain) / this->vehicleParameters.mass;

  accelCommand = accelCommand.cwiseAbs().cwiseMin(this->controllerParameters.maxLinearAcceleration).cwiseProduct(accelCommand.cwiseSign());

  return accelCommand + this->vehicleParameters.gravity;
}

//}

/* ComputeDesiredAngularAcc() //{ */

Eigen::Vector3d LeeVelocityController::ComputeDesiredAngularAcc(const FrameData &_frameData, const EigenTwist &_cmdVel,
                                                                const Eigen::Vector3d &_acceleration) const {
  const Eigen::Matrix3d &rot = _frameData.pose.linear();

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

  Eigen::Vector3d angularRateDes(Eigen::Vector3d::Zero());
  angularRateDes[2] = _cmdVel.angular[2];

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = _frameData.angularVelocityBody - rot.transpose() * rotDes * angularRateDes;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = this->vehicleParameters.inertia;
  // const Eigen::Vector3d &omega = _frameData.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            rot.transpose() * rotDes * angularRateDes);

  // std::cout << moiTerm2.transpose() << std::endl;
  // return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
  //         angularRateError.cwiseProduct(this->normalizedAngularRateGain) +
  //         moiTerm - moiTerm2;
  return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) - angularRateError.cwiseProduct(this->normalizedAngularRateGain);
}

//}

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
