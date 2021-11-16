#include <memory>
#include <optional>
#include <ignition/gazebo/System.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/Model.hh"

#include <sdf/JointAxis.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace cras
{
/// \brief Plugin for rotation of Absolem robot lidar. The lidar rotates back and forth and this plugin handles the
/// rotation direction changes. It thus only receives positive velocity commands and chooses the direction of rotation
/// automatically.
///
/// # Parameters
///
/// `<joint_name>`: Name of the joint that connects the flipper to the track.
///
/// `<topic_vel>`: Custom topic that this system will subscribe to in order to receive velocity commands messages.
/// This element is optional, and the default value is `/model/{name_of_model}/joints/{joint_name}/cmd_vel`.
///
/// `<topic_pos_abs>`: Custom topic that this system will subscribe to in order to receive position commands messages.
/// This element is optional, and the default value is `/model/{name_of_model}/joints/{joint_name}/cmd_pos`.
///
/// `<topic_pos_rel>`: Custom topic that this system will subscribe to in order to receive relative position commands
/// messages. This element is optional, and the default value is
/// `/model/{name_of_model}/joints/{joint_name}/cmd_pos_rel`.
///
/// `<topic_max_torque>`: Custom topic that this system will subscribe to in order to receive torque limit command
/// messages. This element is optional, and the default value is
/// `/model/{name_of_model}/joints/{joint_name}/cmd_max_torque`. This feature is not yet implemented.
///
/// `<max_velocity>`: Maximum angular velocity of the flipper joint that is used in positional and velocity control.
/// Note that this velocity is also limited by the joint's <limit><velocity> setting. Default is 0.5 rad/s.
///
/// `<max_torque>`: Maximum torque the flipper joint can use to reach the given positional or velocity setpoint (or to
/// hold still in its position when stopped). Note that this torque is also limited by the joint's <limit><effort>
/// setting. Default is 30 Nm. This feature is not yet implemented.
///
/// `<position_correction_gain>`: The gain used for positional control. The correcting velocity is computed as
/// gain * (current_position - setpoint) and limited by <max_velocity>. The higher this gain is, the faster will the
/// flipper reach its positional control setpoint. Default is 20.0.
///
/// `<position_correction_tolerance>`: Angular tolerance of positional control (in radians). When the positional error
/// is lower than this threshold, the controller will stop the flipper and try to keep it at the given position. Default
/// is 1 degree.
///
/// # Subscriptions
///
/// `{topic_vel}` (`ignition::msgs::Double`): The desired rotation velocity of the flipper.
/// `{topic_pos_abs}` (`ignition::msgs::Double`): The positional setpoint of the flipper.
/// `{topic_pos_rel}` (`ignition::msgs::Double`): Relative positional setpoint of the flipper.
/// `{topic_max_torque}` (`ignition::msgs::Double`): Maximum torque allowed for the flipper.
class FlipperControlPlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
  public: void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         EntityComponentManager& _ecm, EventManager& _eventMgr) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "FlipperControlPlugin should be attached to a model entity. Failed to initialize." << std::endl;
      return;
    }

    if (!_sdf->HasElement("joint_name"))
    {
      ignerr << "FlipperControlPlugin requires a <joint_name> parameter, but none was given." << std::endl;
      return;
    }

    this->jointName = _sdf->Get<std::string>("joint_name");

    this->joint = model.JointByName(_ecm, this->jointName);
    if (this->joint == kNullEntity)
    {
      ignwarn << "FlipperControlPlugin failed to find flipper joint [" << this->jointName << "] for model ["
              << this->model.Name(_ecm) << "]" << std::endl;
      return;
    }

    const auto jointAxis = _ecm.Component<components::JointAxis>(this->joint)->Data();

    const auto defaultAngularVelocity = jointAxis.MaxVelocity() > 0 ? jointAxis.MaxVelocity() : this->maxAngularVelocity;
    this->maxAngularVelocity = _sdf->Get<double>("max_velocity", defaultAngularVelocity).first;
    if (jointAxis.MaxVelocity() > 0 && this->maxAngularVelocity > jointAxis.MaxVelocity())
    {
      ignwarn << "<max_velocity> of joint " << this->jointName << " set to " << this->maxAngularVelocity
              << " is limited by the joint's <limit><velocity> equal to " << jointAxis.MaxVelocity() << std::endl;
      this->maxAngularVelocity = jointAxis.MaxVelocity();
    }

    const auto defaultMaxTorque = jointAxis.Effort() > 0 ? jointAxis.Effort() : this->maxTorque;
    this->maxTorque = _sdf->Get<double>("max_torque", defaultMaxTorque).first;
    if (jointAxis.Effort() > 0 && this->maxTorque > jointAxis.Effort())
    {
      ignwarn << "<max_torque> of joint " << this->jointName << " set to " << this->maxTorque
              << " is limited by the joint's <limit><effort> equal to " << jointAxis.Effort() << std::endl;
      this->maxTorque = jointAxis.Effort();
    }

    this->positionCorrectionGain = _sdf->Get<double>("position_correction_gain", this->positionCorrectionGain).first;
    this->positionCorrectionTolerance.Radian(
        _sdf->Get<double>(
            "position_correction_tolerance", this->positionCorrectionTolerance.Radian()).first);

    std::string topicTorque {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_max_torque"};
    if (_sdf->HasElement("topic_max_torque"))
      topicTorque = _sdf->Get<std::string>("topic_max_torque");
    this->node.Subscribe(topicTorque, &FlipperControlPlugin::OnCmdTorque, this);

    std::string topicVel {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_vel"};
    if (_sdf->HasElement("topic_vel"))
      topicVel = _sdf->Get<std::string>("topic_vel");
    this->node.Subscribe(topicVel, &FlipperControlPlugin::OnCmdVel, this);

    std::string topicPosAbs {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_pos"};
    if (_sdf->HasElement("topic_pos_abs"))
      topicPosAbs = _sdf->Get<std::string>("topic_pos_abs");
    this->node.Subscribe(topicPosAbs, &FlipperControlPlugin::OnCmdPosAbs, this);

    std::string topicPosRel {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_pos_rel"};
    if (_sdf->HasElement("topic_pos_rel"))
      topicPosRel = _sdf->Get<std::string>("topic_pos_rel");
    this->node.Subscribe(topicPosRel, &FlipperControlPlugin::OnCmdPosRel, this);

    // cached command for flipper joint velocity; the joint has 1 axis, so this vector needs to hold 1 item
    this->velocityCommand.push_back(0.0);

    ignmsg << "FlipperControlPlugin subscribing to cmd_vel messages on [" << topicVel << "] and cmd_pos messages on ["
           << topicPosAbs << "] and cmd_pos_rel messages on [" << topicPosRel << "]. Maximum joint velocity is "
           << this->maxAngularVelocity << " rad/s, maximum joint torque is " << this->maxTorque << " Nm." << std::endl;
  }

  public: void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
  {
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
      ignwarn << "Detected jump back in time ["
              << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
              << "s]. Resetting FlipperControlPlugin." << std::endl;
      this->Reset(_ecm);
      return;
    }

    const auto& angle = _ecm.ComponentDefault<components::JointPosition>(this->joint)->Data()[0];

    if (this->cmdVel.has_value())
    {
      const auto velocity = this->cmdVel.value();
      this->staticAngle.reset();
      if (this->angularSpeed != 0.0 && velocity == 0.0)
      {
        this->staticAngle = angle;
      }
      this->angularSpeed = velocity;
      this->cmdVel.reset();
    } else if (this->cmdPosAbs.has_value()) {
      const auto position = this->cmdPosAbs.value();
      this->staticAngle = position;
      this->cmdPosAbs.reset();
    } else if (this->cmdPosRel.has_value()) {
      const auto position = angle + this->cmdPosRel.value();
      this->staticAngle = position;
      this->cmdPosRel.reset();
    }

    if (this->cmdTorque.has_value()) {
      this->UpdateMaxTorque(this->cmdTorque.value(), _ecm);
      this->cmdTorque.reset();
    }

    this->velocityCommand[0] = this->angularSpeed;
    if (this->staticAngle.has_value())
      this->velocityCommand[0] = this->correctStaticAnglePosition(this->joint, this->staticAngle.value(), _ecm);
    this->velocityCommand[0] = math::clamp(this->velocityCommand[0], -this->maxAngularVelocity, this->maxAngularVelocity);

    _ecm.SetComponentData<components::JointVelocityCmd>(this->joint, this->velocityCommand);
  }

  // To mitigate integrating small velocity errors, if the flipper is said to be stationary, we check that its position
  // does not drift over time.
  protected: double correctStaticAnglePosition(const Entity& joint, const math::Angle& staticPos, EntityComponentManager& _ecm) {
    auto pos = _ecm.Component<components::JointPosition>(this->joint);
    if (!pos || pos->Data().empty())
      return 0.0;

    const math::Angle currentPos{pos->Data()[0]};
    if (fabs((currentPos - staticPos).Radian()) > this->positionCorrectionTolerance.Radian())
    {
      const auto correctingVelocity = this->positionCorrectionGain * (staticPos - currentPos).Radian();
      return correctingVelocity;
    }
    return 0.0;
  }

  protected: void UpdateMaxTorque(const double maxTorqueCmd, EntityComponentManager& _ecm)
  {
    const auto torque = math::clamp(maxTorqueCmd, 0.0, this->maxTorque);
    // TODO max effort cannot be changed during run time, waiting for resolution of
    //  https://github.com/ignitionrobotics/ign-physics/issues/96
    static bool informed{false};
    if (!informed)
    {
      ignwarn << "FlipperControlPlugin: Max torque commands are not yet supported." << std::endl;
      informed = true;
    }
  }

  public: void OnCmdTorque(const msgs::Double &_msg)
  {
    this->cmdTorque = _msg.data();
  }

  public: void OnCmdVel(const msgs::Double &_msg)
  {
    this->cmdVel = _msg.data();
  }

  public: void OnCmdPosAbs(const msgs::Double &_msg)
  {
    this->cmdPosAbs = _msg.data();
  }

  public: void OnCmdPosRel(const msgs::Double &_msg)
  {
    this->cmdPosRel = _msg.data();
  }

  public: void Reset(EntityComponentManager& _ecm)
  {
    this->angularSpeed = 0;
    this->staticAngle.reset();

    if (this->joint != kNullEntity)
    {
      _ecm.SetComponentData<components::JointPosition>(this->joint, {0.0});
      _ecm.SetComponentData<components::JointVelocityCmd>(this->joint, {0.0});
    }

    this->UpdateMaxTorque(this->maxTorque, _ecm);
  }

  protected: Model model{kNullEntity};
  protected: std::string jointName;
  protected: Entity joint{kNullEntity};
  protected: transport::Node node;
  protected: std::optional<double> cmdPosAbs;
  protected: std::optional<double> cmdPosRel;
  protected: std::optional<double> cmdVel;
  protected: std::optional<double> cmdTorque;
  protected: std::optional<ignition::math::Angle> staticAngle{0.0};
  protected: double angularSpeed{0.0};
  protected: double maxTorque{30.0};
  protected: double positionCorrectionGain{20.0};
  protected: math::Angle positionCorrectionTolerance{math::Angle::Pi / 180.0};  // 1 degree
  protected: double maxAngularVelocity{0.5};
  protected: std::vector<double> velocityCommand;
};

}

IGNITION_ADD_PLUGIN(cras::FlipperControlPlugin,
                    System,
                    ISystemConfigure,
                    ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(cras::FlipperControlPlugin, "cras::FlipperControlPlugin")