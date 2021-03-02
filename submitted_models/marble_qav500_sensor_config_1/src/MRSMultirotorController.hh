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

#include "Common.hh"
#include "LeeVelocityController.hh"

#include <ros/package.h>
#include <ros/ros.h>
#include <marble_qav500_sensor_config_1/ControlReference.h>

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

  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &_eventMgr) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  // | -------------------- plugin interface -------------------- |

  void OnTwist(const msgs::Twist &_msg);
  void OnEnable(const msgs::Boolean &_msg);
  void PublishRotorVelocities(ignition::gazebo::EntityComponentManager &_ecm, const Eigen::VectorXd &_vels);

  Model  model{kNullEntity};
  Entity comLinkEntity;

  std::string comLinkName;

  std::string robotNamespace;
  std::string commandSubTopic{"cmd_vel"};
  std::string enableSubTopic{"enable"};

  transport::Node node;

  Eigen::VectorXd rotorVelocities;

  std::unique_ptr<multicopter_control::LeeVelocityController> velocityController;

  multicopter_control::NoiseParameters noiseParameters;

  std::optional<msgs::Twist> cmdVelMsg;

  math::Vector3d maximumLinearVelocity;

  math::Vector3d maximumAngularVelocity;

  std::mutex cmdVelMsgMutex;

  msgs::Actuators rotorVelocitiesMsg;

  bool initialized{false};

  std::atomic<bool> controllerActive{true};

  // | --- subscribing to the special control reference topic --- |

  void                                            callbackControlReference(const marble_qav500_sensor_config_1::ControlReference::ConstPtr &msg);
  ros::Subscriber                                 subscriber_control_reference_;
  bool                                            got_control_reference_ = false;
  marble_qav500_sensor_config_1::ControlReference control_reference_;
};

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
