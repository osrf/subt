/* includes //{ */

#include <SE3Controller.h>

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

/* SE3Controller() //{ */

SE3Controller::SE3Controller(const SE3ControllerParameters &controller_parameters, const VehicleParameters &vehicle_parameters) {

  _controller_parameters_ = controller_parameters;
  _vehicle_parameters_    = vehicle_parameters;

  InitializeParameters();
}

//}

/* CalculateRotorVelocities() //{ */

Eigen::VectorXd SE3Controller::CalculateRotorVelocities(const FrameData &simulator_model_data, const EigenTwist &control_command,
                                                        const SE3ControllerFeedforward &feedforward_command) const {

  // | ----- transform the stuff from the body to the world ----- |

  // rotation from the body to the world
  Eigen::Matrix3d R = simulator_model_data.pose.linear();

  Eigen::Vector3d vel_ref  = R * control_command.linear;
  Eigen::Vector3d acc_ref  = R * feedforward_command.acceleration;
  Eigen::Vector3d jerk_ref = R * feedforward_command.jerk;

  // desired acceleration: proportional control of velocity + acceleration feedforward
  Eigen::Vector3d des_acceleration = ComputeDesiredAcceleration(simulator_model_data, vel_ref, acc_ref);

  const double des_yaw_rate = control_command.angular[2];

  Eigen::Vector3d des_angular_acceleration = SO3Controller(simulator_model_data, des_acceleration, jerk_ref, des_yaw_rate);

  // project thrust onto body z axis.
  double thrust = _vehicle_parameters_.mass * des_acceleration.dot(simulator_model_data.pose.linear().col(2));

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

bool SE3Controller::InitializeParameters() {

  auto allocationMatrix = calculateAllocationMatrix(_vehicle_parameters_.rotorConfiguration);

  if (!allocationMatrix.has_value()) {
    // Error should already be printed by function
    return false;
  }

  // make inertia-independent attitude gain
  normalized_attitude_gain_ = _controller_parameters_.attitude_gain.transpose() * _vehicle_parameters_.inertia.inverse();

  // make inertia-independent attitude rate gain
  normalized_angular_rate_gain_ = _controller_parameters_.angular_rate_gain.transpose() * _vehicle_parameters_.inertia.inverse();

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

Eigen::Vector3d SE3Controller::ComputeDesiredAcceleration(const FrameData &simulator_model_data, const Eigen::Vector3d &vel_ref,
                                                          const Eigen::Vector3d &acc_ref) const {

  // the velocity error
  Eigen::Vector3d velocity_error = vel_ref - simulator_model_data.linearVelocityWorld;

  // feedback on velocity through acceleration
  Eigen::Vector3d feedback_a = velocity_error.cwiseProduct(_controller_parameters_.velocity_gain) / _vehicle_parameters_.mass;

  // sum up the individual components: feedback + feedforward
  Eigen::Vector3d des_accel = feedback_a + acc_ref;

  // --------------------------------------------------------------
  // |       limit the acceleration the the allowed maximum       |
  // --------------------------------------------------------------
  des_accel = des_accel.cwiseAbs().cwiseMin(_controller_parameters_.max_linear_acceleration).cwiseProduct(des_accel.cwiseSign());

  // + gravity compensation
  return des_accel - _vehicle_parameters_.gravity;
}

//}

/* SO3Controller() //{ */

Eigen::Vector3d SE3Controller::SO3Controller(const FrameData &simulator_model_data, const Eigen::Vector3d &des_acceleration, const Eigen::Vector3d &des_jerk,
                                             const double &des_yaw_rate) const {

  // current orientation
  const Eigen::Matrix3d &R = simulator_model_data.pose.linear();

  // desired force acting on the UAV
  Eigen::Vector3d des_f      = des_acceleration * _vehicle_parameters_.mass;
  Eigen::Vector3d des_f_norm = des_f.normalized();

  // | ------- constructing the desired orientation matrix ------ |

  // desired orientation
  Eigen::Matrix3d Rd;

  // b3 will will be equal to the desired force vector
  Eigen::Vector3d b3_des = des_f_norm;

  // since we don't control for desired yaw/heading, we care about maintaning the current one
  // this can be, e.g., approximately achieved by requiring the b1 vector to stay as it is
  Eigen::Vector3d b1_des = R.col(0);

  Rd.col(2) = b3_des;
  Rd.col(1) = Rd.col(2).cross(b1_des);
  Rd.col(1).normalize();
  Rd.col(0) = Rd.col(1).cross(b3_des);
  Rd.col(0).normalize();

  // orientation error
  Eigen::Matrix3d R_e = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  // orientation error vector
  Eigen::Vector3d R_e_vec = vectorFromSkewMatrix(R_e);

  // | -------------------- jerk feedforward -------------------- |

  Eigen::Vector3d w_feedforward = Eigen::Vector3d(0, 0, 0);

  Eigen::Matrix3d I;
  I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
  Eigen::Vector3d desired_jerk = Eigen::Vector3d(des_jerk[0], des_jerk[1], des_jerk[2]);
  w_feedforward                = (I.transpose() * Rd.transpose() * desired_jerk) / (des_acceleration.dot(R.col(2)));

  // | ------------------ angular rate control ------------------ |

  // des angular rate from the feedforward control command
  Eigen::Vector3d des_angular_rate;
  des_angular_rate = Eigen::Vector3d(0, 0, des_yaw_rate) + w_feedforward;

  // angular rate error
  Eigen::Vector3d w_e = simulator_model_data.angularVelocityBody - R.transpose() * Rd * des_angular_rate;

  Eigen::Vector3d des_angular_acceleration = -R_e_vec.cwiseProduct(normalized_attitude_gain_) - w_e.cwiseProduct(normalized_angular_rate_gain_);

  return des_angular_acceleration;
}

//}

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
