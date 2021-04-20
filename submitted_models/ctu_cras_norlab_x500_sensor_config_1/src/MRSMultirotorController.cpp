/* includes //{ */

#include <sdf/sdf.hh>

#include <ignition/gazebo/components/Actuators.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/msgs/actuators.pb.h>
#include <ignition/msgs/twist.pb.h>

#include <ignition/transport/Node.hh>

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Inertial.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <Eigen/Geometry>
#include <memory>

#include <limits>

#include <Common.h>
#include <SE3Controller.h>

//}

/* using //{ */

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace multicopter_control;

//}

/* defines //{ */

#define MAIN_THREAD_RATE 100.0

//}

/* class MRSMultirotorController //{ */

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
class IGNITION_GAZEBO_VISIBLE MRSMultirotorController : public System, public ISystemConfigure, public ISystemPreUpdate {

public:
  MRSMultirotorController() = default;

  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &ecm, EventManager &_eventMgr) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &ecm) override;

private:
  bool is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  math::Vector3d _maximum_linear_velocity_;
  math::Vector3d _maximum_angular_velocity_;

  std::string _robot_namespace_;
  std::string _cmd_vel_topic_{"cmd_vel"};
  std::string _enable_topic_{"enable"};
  std::string _feedforward_topic_{"feedforward"};

  std::string _gazebo_model_entity_name_;

  // noise parameters
  // noise to be added to the UAV states received from the simulator
  multicopter_control::NoiseParameters _noise_parameters_;

  // | -------------------- plugin interface -------------------- |

  void OnTwist(const msgs::Twist &msg);
  void OnEnable(const msgs::Boolean &msg);
  void PublishRotorVelocities(ignition::gazebo::EntityComponentManager &ecm, const Eigen::VectorXd &vels);

  // gazebo links and models
  Model  _gazebo_model_{kNullEntity};
  Entity _gazebo_model_entity_;

  transport::Node ignition_node_;

  Eigen::VectorXd rotor_velocities_;
  msgs::Actuators rotor_velocities_msg_;

  std::unique_ptr<multicopter_control::SE3Controller> multirotor_controller_ptr_;

  std::atomic<bool> is_active_{true};

  // --------------------------------------------------------------
  // |               subscriber to velocity command               |
  // --------------------------------------------------------------

  std::optional<msgs::Twist> cmd_vel_;
  std::mutex                 mutex_cmd_vel_;
  bool cmd_vel_received_ = false;

  // --------------------------------------------------------------
  // |            subscriber for feedforward reference            |
  // --------------------------------------------------------------

  void callbackFeedForward(const msgs::Twist &msg);

  std::optional<msgs::Twist> feedforward_;
  std::mutex                 mutex_feedforward_;
  bool feedforward_received_ = false;
};

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

//}

// | -------------------- plugin interface -------------------- |

/* Configure() //{ */

void MRSMultirotorController::Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf, EntityComponentManager &ecm, EventManager &) {

  int    argc = 0;
  char **argv = NULL;

  ignmsg << "[mrs_multirotor_controller]: initializing" << std::endl;

  {  // gazebo model

    _gazebo_model_ = Model(entity);

    if (!_gazebo_model_.Valid(ecm)) {
      ignerr << "MRSMultirotorController plugin should be attached to a model "
             << "entity. Failed to initialize." << std::endl;
      return;
    }
  }

  auto sdf_clone = sdf->Clone();

  {  // get the com link name

    if (sdf_clone->HasElement("comLinkName")) {
      _gazebo_model_entity_name_ = sdf_clone->Get<std::string>("comLinkName");
    }

    if (_gazebo_model_entity_name_.empty()) {
      ignerr << "found an empty comLinkName parameter. Failed to initialize.\n";
      return;
    }
  }

  {  // get the link entity

    _gazebo_model_entity_ = _gazebo_model_.LinkByName(ecm, _gazebo_model_entity_name_);

    if (_gazebo_model_entity_ == kNullEntity) {
      ignerr << "Link " << _gazebo_model_entity_name_ << " could not be found. Failed to initialize.\n";
      return;
    }
  }

  createFrameDataComponents(ecm, _gazebo_model_entity_);

  VehicleParameters vehicleParams;

  math::Inertiald vehicleInertial;

  // Compute the vehicle's moment of inertia and mass assuming that all the
  // links in the model belong to the vehicle.
  for (const Entity &link : ecm.ChildrenByComponents(_gazebo_model_.Entity(), components::Link())) {
    auto inertial = ecm.Component<components::Inertial>(link);
    if (nullptr == inertial) {
      ignerr << "Could not find inertial component on on link " << _gazebo_model_entity_name_ << std::endl;
      return;
    }
    vehicleInertial += inertial->Data();
  }

  vehicleParams.mass    = vehicleInertial.MassMatrix().Mass();
  vehicleParams.inertia = math::eigen3::convert(vehicleInertial.Moi());

  if (sdf_clone->HasElement("rotorConfiguration")) {
    vehicleParams.rotorConfiguration = loadRotorConfiguration(ecm, sdf_clone->GetElement("rotorConfiguration"), _gazebo_model_, _gazebo_model_entity_);
  } else {
    ignerr << "Please specify rotorConfiguration.\n";
  }

  rotor_velocities_.resize(vehicleParams.rotorConfiguration.size());

  auto worldEntity = ecm.EntityByComponents(components::World());

  if (kNullEntity == worldEntity) {
    ignerr << "World entity missing." << std::endl;
    return;
  }

  // Get the world acceleration (defined in world frame)
  auto gravityComp = ecm.Component<components::Gravity>(worldEntity);

  if (nullptr == gravityComp) {
    ignerr << "World missing gravity." << std::endl;
    return;
  }

  vehicleParams.gravity = math::eigen3::convert(gravityComp->Data());

  SE3ControllerParameters controller_parameters;

  if (sdf_clone->HasElement("velocityGain")) {
    controller_parameters.velocity_gain = math::eigen3::convert(sdf_clone->Get<math::Vector3d>("velocityGain"));
  } else {
    ignerr << "Please specify velocityGain for MRSMultirotorController.\n";
    return;
  }

  if (sdf_clone->HasElement("attitudeGain")) {
    controller_parameters.attitude_gain = math::eigen3::convert(sdf_clone->Get<math::Vector3d>("attitudeGain"));
  } else {
    ignerr << "Please specify attitudeGain for MRSMultirotorController.\n";
    return;
  }

  if (sdf_clone->HasElement("angularRateGain")) {
    controller_parameters.angular_rate_gain = math::eigen3::convert(sdf_clone->Get<math::Vector3d>("angularRateGain"));
  } else {
    ignerr << "Please specify angularRateGain MRSMultirotorController.\n";
    return;
  }

  if (sdf_clone->HasElement("maximumLinearAcceleration")) {
    controller_parameters.max_linear_acceleration = math::eigen3::convert(sdf_clone->Get<math::Vector3d>("maximumLinearAcceleration"));
  } else {
    controller_parameters.max_linear_acceleration.setConstant(std::numeric_limits<double>::max());
  }

  if (sdf_clone->HasElement("_maximum_linear_velocity_")) {
    _maximum_linear_velocity_ = sdf_clone->Get<math::Vector3d>("_maximum_linear_velocity_").Abs();
  } else {
    _maximum_linear_velocity_.Set(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  }

  if (sdf_clone->HasElement("_maximum_angular_velocity_")) {
    _maximum_angular_velocity_ = sdf_clone->Get<math::Vector3d>("_maximum_angular_velocity_").Abs();
  } else {
    _maximum_angular_velocity_.Set(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  }

  multirotor_controller_ptr_ = std::make_unique<multicopter_control::SE3Controller>(controller_parameters, vehicleParams);

  if (nullptr == multirotor_controller_ptr_) {
    ignerr << "Error while creating the LeeVelocityController\n";
    return;
  }

  math::Vector3d linearVelocityMean{0, 0, 0};
  sdf_clone->Get<math::Vector3d>("linearVelocityNoiseMean", linearVelocityMean, linearVelocityMean);

  math::Vector3d linearVelocityStdDev{0, 0, 0};
  sdf_clone->Get<math::Vector3d>("linearVelocityNoiseStdDev", linearVelocityStdDev, linearVelocityStdDev);

  math::Vector3d angularVelocityMean{0, 0, 0};
  sdf_clone->Get<math::Vector3d>("angularVelocityNoiseMean", angularVelocityMean, angularVelocityMean);

  math::Vector3d angularVelocityStdDev{0, 0, 0};
  sdf_clone->Get<math::Vector3d>("angularVelocityNoiseStdDev", angularVelocityStdDev, angularVelocityStdDev);

  _noise_parameters_.linearVelocityMean    = math::eigen3::convert(linearVelocityMean);
  _noise_parameters_.linearVelocityStdDev  = math::eigen3::convert(linearVelocityStdDev);
  _noise_parameters_.angularVelocityMean   = math::eigen3::convert(angularVelocityMean);
  _noise_parameters_.angularVelocityStdDev = math::eigen3::convert(angularVelocityStdDev);

  if (sdf_clone->HasElement("robotNamespace")) {
    _robot_namespace_ = transport::TopicUtils::AsValidTopic(sdf_clone->Get<std::string>("robotNamespace"));
    if (_robot_namespace_.empty()) {
      ignerr << "Robot namespace [" << sdf_clone->Get<std::string>("robotNamespace") << "] is invalid." << std::endl;
      return;
    }
  } else {
    ignerr << "Please specify a robotNamespace.\n";
    return;
  }

  sdf_clone->Get<std::string>("commandSubTopic", _cmd_vel_topic_, _cmd_vel_topic_);
  _cmd_vel_topic_ = transport::TopicUtils::AsValidTopic(_cmd_vel_topic_);
  if (_cmd_vel_topic_.empty()) {
    ignerr << "Invalid command sub-topic." << std::endl;
    return;
  }

  sdf_clone->Get<std::string>("enableSubTopic", _enable_topic_, _enable_topic_);
  _enable_topic_ = transport::TopicUtils::AsValidTopic(_enable_topic_);
  if (_enable_topic_.empty()) {
    ignerr << "Invalid enable sub-topic." << std::endl;
    return;
  }

  // Subscribe to actuator command messages
  std::string topic{_robot_namespace_ + "/" + _cmd_vel_topic_};

  ignition_node_.Subscribe(topic, &MRSMultirotorController::OnTwist, this);
  ignmsg << "MRSMultirotorController subscribing to Twist messages on [" << topic << "]" << std::endl;

  std::string enableTopic{_robot_namespace_ + "/" + _enable_topic_};
  ignition_node_.Subscribe(enableTopic, &MRSMultirotorController::OnEnable, this);
  ignmsg << "MRSMultirotorController subscribing to Boolean messages on [" << enableTopic << "]" << std::endl;

  // Create the Actuators component to take control of rotor speeds
  rotor_velocities_msg_.mutable_velocity()->Resize(rotor_velocities_.size(), 0);

  ecm.CreateComponent(_gazebo_model_.Entity(), components::Actuators(rotor_velocities_msg_));

  // --------------------------------------------------------------
  // |                   custom Ignition subscribers                   |
  // --------------------------------------------------------------

  std::string feedforwardTopic{_robot_namespace_ + "/" + _feedforward_topic_};
  ignition_node_.Subscribe(feedforwardTopic, &MRSMultirotorController::callbackFeedForward, this);
  ignmsg << "MRSMultirotorController subscribing to Twist messages on [" << feedforwardTopic << "]" << std::endl;

  // | ---------------- finish the initialization --------------- |

  ignmsg << "[mrs_multirotor_controller]: initialized" << std::endl;

  is_initialized_ = true;
}

//}

/* PreUpdate() //{ */

void MRSMultirotorController::PreUpdate(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &ecm) {

  IGN_PROFILE("MRSMultirotorController::PreUpdate");

  if (!is_initialized_) {
    return;
  }

  msgs::Twist cmd_vel;

  {
    std::scoped_lock lock(mutex_cmd_vel_);

    if (!cmd_vel_.has_value()) {
      return;
    }

    cmd_vel = cmd_vel_.value();
  }

  msgs::Twist feedforward;

  {
    std::scoped_lock lock(mutex_feedforward_);

    if (feedforward_.has_value()) {
      feedforward = feedforward_.value();
    }
  }

  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time [" << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() << "s]. System may not work properly."
            << std::endl;
  }

  // nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  if (!is_active_) {

    // if the last published rotor velocities were not 0, publish zero velocities
    if (rotor_velocities_.squaredNorm() > 0) {
      rotor_velocities_.setZero();
      this->PublishRotorVelocities(ecm, rotor_velocities_);

      // clear the cmd_vel_ so that the system waits for a new command after being renabled.
      std::lock_guard<std::mutex> lock(mutex_cmd_vel_);
      cmd_vel_.reset();
    }

    return;
  }

  // | ------------- prepare the velocity reference ------------- |

  EigenTwist cmd_vel_eigen;

  // saturate to maximum allowed velocity
  math::Vector3d linear = msgs::Convert(cmd_vel.linear());
  linear.Min(_maximum_linear_velocity_);
  linear.Max(-_maximum_linear_velocity_);

  // saturate to maximum allowed angular velocity
  math::Vector3d angular = msgs::Convert(cmd_vel.angular());
  angular.Min(_maximum_angular_velocity_);
  angular.Max(-_maximum_angular_velocity_);

  cmd_vel_eigen.linear  = math::eigen3::convert(linear);
  cmd_vel_eigen.angular = math::eigen3::convert(angular);

  // | ------------ prepare the feedforward reference ----------- |

  SE3ControllerFeedforward cmd_feedforward;

  cmd_feedforward.acceleration[0] = feedforward.linear().x();
  cmd_feedforward.acceleration[1] = feedforward.linear().y();
  cmd_feedforward.acceleration[2] = feedforward.linear().z();

  cmd_feedforward.jerk[0] = feedforward.angular().x();
  cmd_feedforward.jerk[1] = feedforward.angular().y();
  cmd_feedforward.jerk[2] = feedforward.angular().z();

  // | ------------ get the UAV model simulator data ------------ |

  std::optional<FrameData> frameData = getFrameData(ecm, _gazebo_model_entity_, _noise_parameters_);

  if (!frameData.has_value()) {
    return;
  }

  // | -------------- calculate the control action -------------- |

  rotor_velocities_ = multirotor_controller_ptr_->CalculateRotorVelocities(frameData.value(), cmd_vel_eigen, cmd_feedforward);

  // publish the control action
  PublishRotorVelocities(ecm, rotor_velocities_);
}

//}

/* OnTwist() //{ */

void MRSMultirotorController::OnTwist(const msgs::Twist &msg) {

  {
    std::scoped_lock lock(mutex_cmd_vel_);

    cmd_vel_ = msg;
  }
  
  if (!cmd_vel_received_) {
    ignmsg << "[mrs_multirotor_controller]: MRS controller at work" << std::endl;
    cmd_vel_received_ = true;
  }
}

//}

/* OnEnabled() //{ */

void MRSMultirotorController::OnEnable(const msgs::Boolean &msg) {

  is_active_ = msg.data();
}

//}

/* PublishRotorVelocities() //{ */

void MRSMultirotorController::PublishRotorVelocities(ignition::gazebo::EntityComponentManager &ecm, const Eigen::VectorXd &vels) {

  // check the size of the message
  if (vels.size() != rotor_velocities_msg_.velocity_size()) {
    rotor_velocities_msg_.mutable_velocity()->Resize(vels.size(), 0);
  }

  // fill in the velocities
  for (int i = 0; i < vels.size(); ++i) {
    rotor_velocities_msg_.set_velocity(i, vels(i));
  }

  // Publish the message by setting the Actuators component on the model entity.
  // This assumes that the MulticopterMotorModel system is attached to this model
  auto actuatorMsgComp = ecm.Component<components::Actuators>(_gazebo_model_.Entity());

  if (actuatorMsgComp) {

    auto compFunc = [](const msgs::Actuators &_a, const msgs::Actuators &_b) {
      return std::equal(_a.velocity().begin(), _a.velocity().end(), _b.velocity().begin());
    };

    auto state = actuatorMsgComp->SetData(rotor_velocities_msg_, compFunc) ? ComponentState::PeriodicChange : ComponentState::NoChange;

    ecm.SetChanged(_gazebo_model_.Entity(), components::Actuators::typeId, state);

  } else {
    ecm.CreateComponent(_gazebo_model_.Entity(), components::Actuators(rotor_velocities_msg_));
  }
}

//}

// | --------------------- other routines --------------------- |

/* callbackFeedForward() //{ */

void MRSMultirotorController::callbackFeedForward(const msgs::Twist &msg) {

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_feedforward_);

    feedforward_ = msg;
  }

  if (!feedforward_received_) {
    ignmsg << "[mrs_multirotor_controller]: getting feedforward reference"
      << std::endl;
    feedforward_received_ = true;
  }
}

//}

//}

IGNITION_ADD_PLUGIN(MRSMultirotorController, ignition::gazebo::System, MRSMultirotorController::ISystemConfigure, MRSMultirotorController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MRSMultirotorController, "ignition::gazebo::systems::MRSMultirotorController")
