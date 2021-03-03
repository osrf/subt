#include <memory>
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
/// `<joint_name>`: Name of the joint that connects the lidar to the body.
///
/// `<topic>`: Custom topic that this system will subscribe to in order to receive velocity commands messages.
/// This element is optional, and the default value is `/model/{name_of_model}/joints/{joint_name}/cmd_vel`.
///
/// `<rotation_angular_limit>`: The angle at which laser rotation direction should change. Zero is when the scanning
/// plane is perpendicular to ground.
///
/// `<max_velocity>`: Maximum angular velocity of the lidar joint.
///
/// `<initial_velocity>`: If set, the lidar will start rotating with this velocity right after the model is spawned.
///
/// # Subscriptions
///
/// `{topic}` (`ignition::msgs::Double`): The desired scanning speed. Only positive values are expected. All values
/// higher than `{max_velocity}` will be clamped.
class LaserRotatePlugin : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
{
  public: void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         EntityComponentManager& _ecm, EventManager& _eventMgr) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "LaserRotatePlugin should be attached to a model entity. Failed to initialize." << std::endl;
      return;
    }

    this->jointName = _sdf->Get<std::string>("joint_name", "laser_j").first;

    this->rotationAngularLimit = _sdf->Get<double>("rotation_angular_limit", this->rotationAngularLimit).first;
    this->maxAngularVelocity = _sdf->Get<double>("max_velocity", this->maxAngularVelocity).first;

    if (_sdf->HasElement("initial_velocity"))
    {
      this->initialScanningSpeed = _sdf->Get<double>("initial_velocity");
      this->lastCmdVel = this->initialScanningSpeed;
    }

    std::string topic {"/model/" + this->model.Name(_ecm) + "/joint/" + this->jointName + "/cmd_vel"};
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");
    this->node.Subscribe(topic, &LaserRotatePlugin::OnCmdVel, this);

    ignmsg << "LaserRotatePlugin subscribing to cmd_vel messages on [" << topic << "]" << std::endl;
  }

  public: void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
  {
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
      ignwarn << "Detected jump back in time ["
              << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
              << "s]. Resetting LaserRotatePlugin." << std::endl;
      this->Reset(_ecm);
      return;
    }

    if (this->joint == kNullEntity)
    {
      this->joint = this->model.JointByName(_ecm, this->jointName);
      if (this->joint == kNullEntity)
      {
        ignwarn << "Failed to find laser joint [" << this->jointName << "] for model [" << this->model.Name(_ecm) << "]"
                << std::endl;
        return;
      }
    }

    auto& jointPosition = _ecm.ComponentDefault<components::JointPosition>(this->joint)->Data().at(0);

    if (this->rotationVelocitySigned == 0.0)
    {
      jointPosition = this->staticAngle;
    }
    else if (jointPosition >= rotationAngularLimit)
    {
      this->rotationVelocitySigned = -std::abs(this->rotationVelocitySigned);
    }
    else if (jointPosition <= -rotationAngularLimit)
    {
      this->rotationVelocitySigned = std::abs(this->rotationVelocitySigned);
    }

    _ecm.SetComponentData<components::JointVelocityCmd>(this->joint, {this->rotationVelocitySigned});
  }

  public: void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
  {
    this->setScanningSpeed(this->lastCmdVel, _ecm);
  }

  protected: void setRotationVelocitySigned(const double speed, const EntityComponentManager& _ecm)
  {
    auto sanitizedSpeed = speed;
    if (std::abs(speed) < 1e-3)
    {
      sanitizedSpeed = 0;
      if (this->joint != kNullEntity)
      {
        const auto jointPos = _ecm.Component<components::JointPosition>(this->joint);
        if (jointPos != nullptr && !jointPos->Data().empty())
          this->staticAngle = jointPos->Data()[0];
      }
    }

    // if the laser is being stopped, save the motion direction
    const auto oldSpeed = this->rotationVelocitySigned;
    if (oldSpeed != 0.0 && sanitizedSpeed == 0.0)
    {
      this->lastVelocitySign = (this->rotationVelocitySigned > 0) ? 1 : -1;
    }

    if (sanitizedSpeed > 0)
    {
      this->rotationVelocitySigned = std::min(sanitizedSpeed, this->maxAngularVelocity);
    }
    else
    {
      this->rotationVelocitySigned = std::max(sanitizedSpeed, -this->maxAngularVelocity);
    }

    // keep rotation direction when we start the motion from a stopped state
    if (oldSpeed == 0.0 && sanitizedSpeed != 0.0)
    {
      this->rotationVelocitySigned = this->lastVelocitySign * this->rotationVelocitySigned;
    }
  }

  protected: void setScanningSpeed(const double speed, const EntityComponentManager& _ecm)
  {
    // scanning speed can only be positive
    const auto sanitizedSpeed = std::max(0.0, speed);
    const auto currentVelocitySign = (this->rotationVelocitySigned >= 0) ? 1 : -1;

    // but we want to keep the current rotation direction
    this->setRotationVelocitySigned(sanitizedSpeed * currentVelocitySign, _ecm);
  }

  public: void OnCmdVel(const msgs::Double &_msg)
  {
    this->lastCmdVel = _msg.data();
  }

  public: void Reset(EntityComponentManager& _ecm)
  {
    this->setRotationVelocitySigned(this->initialScanningSpeed, _ecm);
    this->staticAngle = 0.0;
    this->lastVelocitySign = 1;

    if (this->joint != kNullEntity)
    {
      _ecm.SetComponentData<components::JointPosition>(this->joint, {0.0});
      _ecm.SetComponentData<components::JointVelocityCmd>(this->joint, {this->initialScanningSpeed});
    }
  }

  protected: Model model{kNullEntity};
  protected: std::string jointName;
  protected: Entity joint{kNullEntity};
  protected: transport::Node node;
  protected: double lastCmdVel{0.0};
  protected: double rotationVelocitySigned{0.0};
  protected: double staticAngle{0.0};
  protected: int lastVelocitySign{1};  // stores motion direction when velocity is set to 0
  protected: double initialScanningSpeed{0};
  protected: double maxAngularVelocity{1.2};
  protected: double rotationAngularLimit{1.618994};
};

}

IGNITION_ADD_PLUGIN(cras::LaserRotatePlugin,
                    System,
                    ISystemConfigure,
                    ISystemPreUpdate,
                    ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(cras::LaserRotatePlugin, "cras::LaserRotatePlugin")