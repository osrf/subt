#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_HH_

/* includes //{ */

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include <Common.hh>

#include <ros/package.h>
#include <ros/ros.h>
#include <marble_qav500_sensor_config_1/ControlReference.h>

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

  std::unique_ptr<multicopter_control::BacaSE3Controller> multirotor_controller_ptr_;

  std::atomic<bool> is_active_{true};

  // --------------------------------------------------------------
  // |               subscriber to velocity command               |
  // --------------------------------------------------------------

  std::optional<msgs::Twist> cmd_vel_;
  std::mutex                 mutex_cmd_vel_;

  // --------------------------------------------------------------
  // |            subscriber for feedforward reference            |
  // --------------------------------------------------------------

  void callbackFeedForward(const marble_qav500_sensor_config_1::ControlReferenceConstPtr &msg);

  ros::Subscriber                                                subscriber_feedforward_;
  std::optional<marble_qav500_sensor_config_1::ControlReference> feedforward_;
  std::mutex                                                     mutex_feedforward_;
};

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
