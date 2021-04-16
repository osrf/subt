/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * Copyright (C) 2021 Czech Technical University in Prague
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/pid.pb.h>

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Model.hh"

#include <memory>
#include <ignition/gazebo/System.hh>

using namespace ignition;
using namespace gazebo;

namespace subt
{
/// \brief Joint position controller which can be attached to a model with a
/// reference to a single joint. This is an improved version of the standard
/// Ignition Gazebo JointPositionController system which allows for runtime
/// changes of the PID parameters and also allows to specify an initial target.
///
/// A new Ignition Transport topic is created to send target joint positions.
/// The topic name is
/// "/model/<model_name>/joint/<joint_name>/<joint_index>/cmd_pos"
///
/// This topic accepts ignition::msgs::Double values representing the target
/// position. Sending NaN will disable the controller until another finite
/// command is issued. Be aware that sending NaNs does not work through
/// the MultiJointCommandSystem and you have to send the NaNs to each controller
/// separately.
///
/// The following service is advertised:
/// "/model/<model_name>/joint/<joint_name>/<joint_index>/set_pos_pid"
///
/// That service accepts ignition::msgs::PID messages that configure the internal PID.
/// The system reads only the new _optional fields of the message, not the deprecated ones.
///
/// ## System Parameters
///
/// `<topic>` If set, changes the command topic. Optional parameter.
///
/// `<pid_topic>` If set, changes the PID configuration topic. Optional parameter.
///
/// `<joint_name>` The name of the joint to control. Required parameter.
///
/// `<joint_index>` Axis of the joint to control. Optional parameter.
///  The default value is 0.
///
/// `<initial_target_pos>` Optional parameter specifying the target position of
///  the joint before any commands arrive. If omitted, the joint will not be
///  controlled until commanded (see <keep_initial_pos>, though).
///
/// `<keep_initial_pos>` Optional bool parameter specifying that the joint should
///  be controlled to keep its initial position until commanded otherwise.
///  If <initial_target_pos> is also set, this parameter is ignored and the
///  joint is controlled to reach the position from <initial_target_pos>.
///
/// `<p_gain>` The proportional gain of the PID. Optional parameter.
///  The default value is 1.
///
/// `<i_gain>` The integral gain of the PID. Optional parameter.
///  The default value is 0.1.
///
/// `<d_gain>` The derivative gain of the PID. Optional parameter.
///  The default value is 0.01
///
/// `<i_max>` The integral upper limit of the PID. Optional parameter.
///  The default value is 1.
///
/// `<i_min>` The integral lower limit of the PID. Optional parameter.
///  The default value is -1.
///
/// `<cmd_max>` Output max value of the PID. Optional parameter.
///  The default value is 1000.
///
/// `<cmd_min>` Output min value of the PID. Optional parameter.
///  The default value is -1000.
///
/// `<cmd_offset>` Command offset (feed-forward) of the PID. Optional
/// parameter. The default value is 0.
class ConfigurableJointPositionController : public System, public ISystemConfigure, public ISystemPreUpdate
{
  public: void Configure(
    const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm, EventManager& _eventMgr) override;

  public: void PreUpdate(
    const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm) override;

  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  protected: void OnCmdPos(const ignition::msgs::Double& _msg);

  /// \brief Callback for PID config
  /// \param[in] _msg PID message
  protected: void OnCmdPID(const ignition::msgs::PID& _msg);

  protected: transport::Node node;
  protected: Entity jointEntity{kNullEntity};
  protected: std::string jointName;
  protected: std::optional<double> jointPosCmd;
  protected: std::mutex jointCmdMutex;
  protected: Model model{kNullEntity};
  protected: std::string modelName;
  protected: math::PID posPid;
  protected: unsigned int jointIndex {0u};
  protected: bool keepInitialPos {false};
};

void ConfigurableJointPositionController::Configure(
  const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm, EventManager&/*_eventMgr*/)
{
  this->model = Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    ignerr << "ConfigurableJointPositionController plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  this->modelName = this->model.Name(_ecm);

  // Get params from SDF
  this->jointName = _sdf->Get<std::string>("joint_name");

  if (this->jointName.empty())
  {
    ignerr << "ConfigurableJointPositionController found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (_sdf->HasElement("joint_index"))
  {
    this->jointIndex = _sdf->Get<unsigned int>("joint_index");
  }

  // PID parameters
  double p = 1;
  double i = 0.1;
  double d = 0.01;
  double iMax = 1;
  double iMin = -1;
  double cmdMax = 1000;
  double cmdMin = -1000;
  double cmdOffset = 0;

  if (_sdf->HasElement("p_gain"))
  {
    p = _sdf->Get<double>("p_gain");
  }
  if (_sdf->HasElement("i_gain"))
  {
    i = _sdf->Get<double>("i_gain");
  }
  if (_sdf->HasElement("d_gain"))
  {
    d = _sdf->Get<double>("d_gain");
  }
  if (_sdf->HasElement("i_max"))
  {
    iMax = _sdf->Get<double>("i_max");
  }
  if (_sdf->HasElement("i_min"))
  {
    iMin = _sdf->Get<double>("i_min");
  }
  if (_sdf->HasElement("cmd_max"))
  {
    cmdMax = _sdf->Get<double>("cmd_max");
  }
  if (_sdf->HasElement("cmd_min"))
  {
    cmdMin = _sdf->Get<double>("cmd_min");
  }
  if (_sdf->HasElement("cmd_offset"))
  {
    cmdOffset = _sdf->Get<double>("cmd_offset");
  }

  this->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  this->keepInitialPos = _sdf->HasElement("keep_initial_pos");

  if (_sdf->HasElement("initial_target_pos"))
  {
    this->jointPosCmd = _sdf->Get<double>("initial_target_pos");
    if (this->keepInitialPos)
      ignwarn << "Position controller of joint [" << this->jointName << "] has "
                 "both <initial_target_pos> and <keep_initial_pos> specified. "
                 "In this case, <keep_initial_pos> is ignored." << std::endl;
  }

  // Subscribe to commands
  std::string topic = transport::TopicUtils::AsValidTopic("/model/" +
    this->model.Name(_ecm) + "/joint/" + this->jointName +
    "/" + std::to_string(this->jointIndex) + "/cmd_pos");
  if (topic.empty() && !_sdf->HasElement("topic"))
  {
    ignerr << "Failed to create topic for joint [" << this->jointName << "]" << std::endl;
    return;
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << this->jointName << "]" << std::endl;
      return;
    }
  }

  std::string pidTopic = transport::TopicUtils::AsValidTopic("/model/" +
    this->model.Name(_ecm) + "/joint/" + this->jointName +
    "/" + std::to_string(this->jointIndex) + "/set_pos_pid");
  if (pidTopic.empty() && !_sdf->HasElement("pid_topic"))
  {
    ignerr << "Failed to create PID service for joint [" << this->jointName << "]" << std::endl;
    return;
  }
  if (_sdf->HasElement("pid_topic"))
  {
    pidTopic = transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>("pid_topic"));

    if (pidTopic.empty())
    {
      ignerr << "Failed to create PID service [" << _sdf->Get<std::string>("pid_topic")
             << "]" << " for joint [" << this->jointName << "]" << std::endl;
      return;
    }
  }

  if (topic == pidTopic)
  {
    ignerr << "Command and PID topics are the same for joint [" << this->jointName << "]" << std::endl;
    return;
  }

  this->node.Subscribe(topic, &ConfigurableJointPositionController::OnCmdPos, this);
  this->node.Advertise(pidTopic, &ConfigurableJointPositionController::OnCmdPID, this);

  igndbg << "[ConfigurableJointPositionController] system parameters:" << std::endl;
  igndbg << "p_gain: [" << p << "]" << std::endl;
  igndbg << "i_gain: [" << i << "]" << std::endl;
  igndbg << "d_gain: [" << d << "]" << std::endl;
  igndbg << "i_max: [" << iMax << "]" << std::endl;
  igndbg << "i_min: [" << iMin << "]" << std::endl;
  igndbg << "cmd_max: [" << cmdMax << "]" << std::endl;
  igndbg << "cmd_min: [" << cmdMin << "]" << std::endl;
  igndbg << "cmd_offset: [" << cmdOffset << "]" << std::endl;
  if (this->jointPosCmd.has_value())
    igndbg << "Initial target: [" << this->jointPosCmd.value() << "]" << std::endl;
  else if (this->keepInitialPos)
    igndbg << "Initial target: [initial position]" << std::endl;
  else
    igndbg << "Initial target: none" << std::endl;
  igndbg << "Topic: [" << topic << "]" << std::endl;
  igndbg << "PID config service: [" << pidTopic << "]" << std::endl;
}

void ConfigurableJointPositionController::PreUpdate(
  const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  IGN_PROFILE("ConfigurableJointPositionController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
  if (this->jointEntity == kNullEntity)
    this->jointEntity = this->model.JointByName(_ecm, this->jointName);

  if (this->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // no command received yet
  if (!this->jointPosCmd.has_value() && !this->keepInitialPos)
    return;

  auto jointPosComp = _ecm.ComponentDefault<components::JointPosition>(this->jointEntity);

  // Sanity check: Make sure that the joint index is valid.
  if (this->jointIndex >= jointPosComp->Data().size())
  {
    static bool invalidJointReported = false;
    if (!invalidJointReported)
    {
      ignerr << "[ConfigurableJointPositionController]: Detected an invalid <joint_index> "
             << "parameter. The index specified is [" << this->jointIndex << "] but the joint only has ["
             << jointPosComp->Data().size() << "] index[es]. This controller will be ignored" << std::endl;
      invalidJointReported = true;
    }
    return;
  }

  // read initial joint position if we should keep it
  if (this->keepInitialPos && !this->jointPosCmd.has_value())
    this->jointPosCmd = jointPosComp->Data().at(this->jointIndex);

  // no command yet and we should not keep the initial position
  if (!this->jointPosCmd.has_value())
    return;

  // sending NaN disables the controller
  if (!std::isfinite(this->jointPosCmd.value()))
    return;

  // Update force command.
  double error;
  {
    std::lock_guard<std::mutex> lock(this->jointCmdMutex);
    error = jointPosComp->Data().at(this->jointIndex) - this->jointPosCmd.value();
  }

  double force = this->posPid.Update(error, _info.dt);

  auto forceCmd = _ecm.ComponentDefault<components::JointForceCmd>(this->jointEntity)->Data();
  forceCmd.resize(std::max(forceCmd.size(), static_cast<size_t>(this->jointIndex + 1)));
  forceCmd[this->jointIndex] = force;

  _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity, forceCmd);
}

void ConfigurableJointPositionController::OnCmdPos(const msgs::Double& _msg)
{
  std::lock_guard<std::mutex> lock(this->jointCmdMutex);
  this->jointPosCmd = _msg.data();
}

void ConfigurableJointPositionController::OnCmdPID(const msgs::PID& _msg)
{
  std::lock_guard<std::mutex> lock(this->jointCmdMutex);

  if (_msg.has_p_gain_optional())
    this->posPid.SetPGain(_msg.p_gain_optional().data());
  if (_msg.has_i_gain_optional())
    this->posPid.SetIGain(_msg.i_gain_optional().data());
  if (_msg.has_d_gain_optional())
    this->posPid.SetDGain(_msg.d_gain_optional().data());
  if (_msg.has_i_max_optional())
    this->posPid.SetIMax(_msg.i_max_optional().data());
  if (_msg.has_i_min_optional())
    this->posPid.SetIMin(_msg.i_min_optional().data());
  if (_msg.has_limit_optional())
    this->posPid.SetIMin(_msg.i_min_optional().data());

  const auto& stamp = _msg.header().stamp();
  const auto now = std::chrono::duration_cast<std::chrono::duration<float>>(
    math::secNsecToDuration(stamp.sec(), stamp.nsec()));
  igndbg << "[" << std::fixed << std::setprecision(3) << now.count() << "]: "
         << "Updated PID properties of joint ["
         << this->modelName << "/" << this->jointName << "]" << std::endl;
}

}

IGNITION_ADD_PLUGIN(subt::ConfigurableJointPositionController,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(subt::ConfigurableJointPositionController,
                         "subt::ConfigurableJointPositionController")
