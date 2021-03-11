#include <memory>
#include <optional>
#include <ignition/gazebo/System.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

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
/// `<topic_pos>`: Custom topic that this system will subscribe to in order to receive position commands messages.
/// This element is optional, and the default value is `/model/{name_of_model}/joints/{joint_name}/cmd_pos`.
///
/// `<max_velocity>`: Maximum angular velocity of the flipper joint that is used in positional control.
///
/// # Subscriptions
///
/// `{topic_vel}` (`ignition::msgs::Double`): The desired rotation velocity of the flipper.
/// `{topic_pos}` (`ignition::msgs::Double`): The positional setpoint of the flipper.
class FlipperControlPlugin : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
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

    this->maxAngularVelocity = _sdf->Get<double>("max_velocity", this->maxAngularVelocity).first;
    this->maxTorque = _sdf->Get<double>("max_torque", this->maxTorque).first;
    this->positionCorrectionGain = _sdf->Get<double>("position_correction_gain", this->positionCorrectionGain).first;

    std::string topicTorque {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_torque"};
    if (_sdf->HasElement("topic_torque"))
      topicTorque = _sdf->Get<std::string>("topic_torque");
    this->node.Subscribe(topicTorque, &FlipperControlPlugin::OnCmdTorque, this);

    std::string topicVel {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_vel"};
    if (_sdf->HasElement("topic_vel"))
      topicVel = _sdf->Get<std::string>("topic_vel");
    this->node.Subscribe(topicVel, &FlipperControlPlugin::OnCmdVel, this);

    std::string topicPos {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_pos"};
    if (_sdf->HasElement("topic_pos"))
      topicPos = _sdf->Get<std::string>("topic_pos");
    this->node.Subscribe(topicPos, &FlipperControlPlugin::OnCmdPos, this);

    ignmsg << "FlipperControlPlugin subscribing to cmd_vel messages on [" << topicVel << "] and cmd_pos messages on ["
      << topicPos << "]" << std::endl;
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

    if (this->joint == kNullEntity)
    {
      this->joint = this->model.JointByName(_ecm, this->jointName);
      if (this->joint == kNullEntity)
      {
        ignwarn << "Failed to find flipper joint [" << this->jointName << "] for model [" << this->model.Name(_ecm)
          << "]" << std::endl;
        return;
      }
    }

    auto pos = _ecm.Component<components::JointPosition>(this->joint);
    if (!pos)
      _ecm.CreateComponent(this->joint, components::JointPosition());

    auto vel = _ecm.Component<components::JointVelocityCmd>(this->joint);
    if (!vel)
    {
      _ecm.CreateComponent(this->joint, components::JointVelocityCmd({this->angularSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({this->angularSpeed});
    }

    if (this->angularSpeed == 0.0)
      this->correctStaticAnglePosition(this->joint, this->staticAngle, _ecm);
  }

  public: void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
  {
    if (this->cmdVel.has_value())
    {
      const auto velocity = this->cmdVel.value();
      if (this->joint != kNullEntity) {
        if (this->angularSpeed != 0.0 && velocity == 0.0)
        {
          auto pos = _ecm.Component<components::JointPosition>(this->joint);
          if (pos && !pos->Data().empty())
          {
            this->staticAngle = pos->Data()[0];
          }
        }
        const auto sanitizedVelocity = math::clamp(velocity, -this->maxAngularVelocity, this->maxAngularVelocity);
        this->angularSpeed = sanitizedVelocity;
      }
      this->cmdVel.reset();
    } else if (this->cmdPos.has_value()) {
      const auto position = this->cmdPos.value();
      if (this->joint != kNullEntity) {
        this->staticAngle = position;
      }
      this->cmdPos.reset();
    }

    if (this->cmdTorque.has_value()) {
      this->UpdateMaxTorque(this->cmdTorque.value(), _ecm);
      this->cmdTorque.reset();
    }
  }

  // To mitigate integrating small velocity errors, if the flipper is said to be stationary, we check that its position
  // does not drift over time.
  protected: void correctStaticAnglePosition(const Entity& joint, const math::Angle& staticPos, EntityComponentManager& _ecm) {
    auto pos = _ecm.Component<components::JointPosition>(this->joint);
    if (!pos || pos->Data().empty())
      return;

    const math::Angle currentPos{pos->Data()[0]};
    if (fabs((currentPos - staticPos).Degree()) > 1.0)
    {
      const auto correctingVelocity = this->positionCorrectionGain * (staticPos - currentPos).Radian();
      const auto sanitizedVelocity = math::clamp(correctingVelocity, -this->maxAngularVelocity, this->maxAngularVelocity);
      auto vel = _ecm.Component<components::JointVelocityCmd>(this->joint);
      if (!vel)
      {
        _ecm.CreateComponent(this->joint, components::JointVelocityCmd({sanitizedVelocity}));
      }
      else
      {
        *vel = components::JointVelocityCmd({sanitizedVelocity});
      }
    }
  }

  protected: void UpdateMaxTorque(const double maxTorque, const EntityComponentManager& _ecm)
  {
    // TODO
  }

  public: void OnCmdTorque(const msgs::Double &_msg)
  {
    this->cmdTorque = _msg.data();
  }

  public: void OnCmdVel(const msgs::Double &_msg)
  {
    this->cmdVel = _msg.data();
  }

  public: void OnCmdPos(const msgs::Double &_msg)
  {
    this->cmdPos = _msg.data();
  }

  public: void Reset(EntityComponentManager& _ecm)
  {
    this->angularSpeed = 0;
    this->staticAngle = ignition::math::Angle(0);

    if (this->joint != kNullEntity)
    {
      auto pos = _ecm.Component<components::JointPosition>(this->joint);
      if (pos)
      {
        *pos = components::JointPosition({0.0});
      }

      auto vel = _ecm.Component<components::JointVelocityCmd>(this->joint);
      if (vel)
      {
        *vel = components::JointVelocityCmd({0.0});
      }
    }

    this->UpdateMaxTorque(this->maxTorque, _ecm);
  }

  protected: Model model{kNullEntity};
  protected: std::string jointName;
  protected: Entity joint{kNullEntity};
  protected: transport::Node node;
  protected: std::optional<double> cmdPos;
  protected: std::optional<double> cmdVel;
  protected: std::optional<double> cmdTorque;
  protected: ignition::math::Angle staticAngle{0.0};
  protected: double angularSpeed{0.0};
  protected: double maxTorque{30.0};
  protected: double positionCorrectionGain{0.2};
  protected: double maxAngularVelocity{0.5};
};

}

IGNITION_ADD_PLUGIN(cras::FlipperControlPlugin,
                    System,
                    ISystemConfigure,
                    ISystemPreUpdate,
                    ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(cras::FlipperControlPlugin, "cras::FlipperControlPlugin")
