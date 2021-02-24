#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/common/StringUtils.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace subt
{
/// \brief Plugin for controlling multiple joints at once. This should avoid time-sync
/// problems in control algorithms.
///
/// This plugin does not control the joints itself. The joints need to spawn their own
/// positional, velocity or effort controllers that accept commands on Ignition topics.
/// This plugin just de-multiplexes the single control command into multiple commands for the controllers.
/// This relaying should happen during a single simulation step.
///
/// Only finite values will be published to the controllers. So if you e.g. only want to set
/// the effort target for a joint, be sure to set the position and velocity targets to NaN,
/// otherwise this plugin will set all three targets (even if the other two are zero).
/// This might be no problem if there actually are none positional and velocity controllers
/// for the joint - then the commands will be published, but nobody will read them.
///
/// Once https://github.com/ignitionrobotics/ros_ign/pull/135 gets merged, it will
/// be possible to e.g. leave the position and velocity fields empty and only fill out
/// name and effort.
///
/// # Parameters
///
/// `<topic>`: Name of the topic on which this system receives commands. Defaults to `/model/${MODEL_NAME}/joint_commands`.
/// `<joint name="test_joint">`: The plugin is configured by a list of <joint> tags, each of which specifies one controlled joint.
///                              The `name` attribute is mandatory and specifies the name of the joint.
///
/// `<joint><position_target_topic>`: Topic for sending positional commands. Defaults to `/model/${MODEL_NAME}/joint/${JOINT_NAME}/cmd_pos`.
/// `<joint><velocity_target_topic>`: Topic for sending velocity commands. Defaults to `/model/${MODEL_NAME}/joint/${JOINT_NAME}/cmd_vel`.
/// `<joint><effort_target_topic>`: Topic for sending effort commands. Defaults to `/model/${MODEL_NAME}/joint/${JOINT_NAME}/cmd_force`.
///
/// # Subscriptions
///
/// `{topic}` (`ignition::msgs::Model`): The system receives the multi-commands on this topic. It only parses the
/// fields that are present, and is only interested in the header and joints/axis1 fields. Ignition-ROS bridge
/// provides a converter between sensor_msgs/JointState and ignition::msgs::Model. Make sure the ROS message
/// has all four vectors of the same length, otherwise the Ign-ROS bridge could segfault!
class MultiJointCommandSystem : public System, public ISystemConfigure, public ISystemPreUpdate
{
  public: void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         EntityComponentManager& _ecm, EventManager& _eventMgr) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "MultiJointCommandSystem should be attached to a model entity. Failed to initialize." << std::endl;
      return;
    }

    if (!_sdf->HasElement("joint"))
    {
      ignerr << "MultiJointCommandSystem doesn't have any <joint> element. It will not do anything." << std::endl;
      return;
    }

    auto sdf = _sdf->Clone();

    auto joint = sdf->GetElement("joint");
    while (joint)
    {
      if (!joint->HasAttribute("name"))
      {
        ignerr << "MultiJointCommandSystem found a <joint> element with no `name` attribute. This element will be ignored." << std::endl;
        joint = joint->GetNextElement("joint");
        continue;
      }

      const auto name = joint->GetAttribute("name")->GetAsString();

      const auto& jointEntity = this->model.JointByName(_ecm, name);
      if (jointEntity == kNullEntity)
      {
        ignerr << "MultiJointCommandSystem tried to configure for joint " << name << " but that joint does not exist in the model. It will be ignored." << std::endl;
        joint = joint->GetNextElement("joint");
        continue;
      }

      const auto prefix = "/model/" + this->model.Name(_ecm) + "/joint/" + name;

      std::string posTopic = prefix + "/cmd_pos";
      std::string velTopic = prefix + "/cmd_vel";
      std::string forceTopic = prefix + "/cmd_force";

      if (sdf->HasElement("position_target_topic"))
        posTopic = sdf->Get<std::string>("position_target_topic");
      if (sdf->HasElement("velocity_target_topic"))
        velTopic = sdf->Get<std::string>("velocity_target_topic");
      if (sdf->HasElement("effort_target_topic"))
        forceTopic = sdf->Get<std::string>("effort_target_topic");

      const auto prevPosTopic = posTopic;
      posTopic = transport::TopicUtils::AsValidTopic(posTopic);
      if (posTopic.empty())
        ignwarn << "Invalid topic name " << prevPosTopic;
      else
      {
        this->posPublishers[name] = this->node.Advertise<msgs::Double>(posTopic);
        if (posTopic != prevPosTopic)
          ignwarn << "Topic name adjusted from " << prevPosTopic << " to " << posTopic;
      }
      
      const auto prevVelTopic = velTopic;
      velTopic = transport::TopicUtils::AsValidTopic(velTopic);
      if (velTopic.empty())
        ignwarn << "Invalid topic name " << prevVelTopic;
      else
      {
        this->velPublishers[name] = this->node.Advertise<msgs::Double>(velTopic);
        if (velTopic != prevVelTopic)
          ignwarn << "Topic name adjusted from " << prevVelTopic << " to " << velTopic;
      }
      
      const auto prevForceTopic = forceTopic;
      forceTopic = transport::TopicUtils::AsValidTopic(forceTopic);
      if (forceTopic.empty())
        ignwarn << "Invalid topic name " << prevForceTopic;
      else
      {
        this->forcePublishers[name] = this->node.Advertise<msgs::Double>(forceTopic);
        if (forceTopic != prevForceTopic)
          ignwarn << "Topic name adjusted from " << prevForceTopic << " to " << forceTopic;
      }
      
      ignmsg << "MultiJointControlSystem was configured for joint " << name << std::endl;

      joint = joint->GetNextElement("joint");
    }

    std::string topic {"/model/" + this->model.Name(_ecm) + "/joint_commands"};
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");

    this->node.Subscribe(topic, &MultiJointCommandSystem::OnCmd, this);

    ignmsg << "MultiJointCommandSystem subscribing to commands on [" << topic << "]" << std::endl;

    this->initialized = true;
  }

  public: void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
  {
    if (!this->initialized)
      return;
    std::vector<std::string> sentCommands;
    std::lock_guard<std::mutex> lock(this->commandLock);
    for (const auto& joint : this->command.joint())
    {
      if (joint.name().empty())
      {
        ignwarn << "Received command with empty joint name." << std::endl;
        continue;
      }

      const auto& name = joint.name();

      if (!joint.has_axis1())
      {
        ignwarn << "Received command for joint " << name << " without axis1." << std::endl;
        continue;
      }

      const auto& axis = joint.axis1();

      if (std::isfinite(axis.position()))
      {
        if (this->posPublishers.find(name) != this->posPublishers.end())
        {
          msgs::Double msg;
          *msg.mutable_header() = this->command.header();
          msg.set_data(axis.position());
          this->posPublishers[name].Publish(msg);
          sentCommands.push_back(name + "[pos]");
        }
        else
          ignwarn << "Received positional command for joint " << name << " which doesn't have a valid positional target topic." << std::endl;
      }

      if (std::isfinite(axis.velocity()))
      {
        if (this->velPublishers.find(name) != this->velPublishers.end())
        {
          msgs::Double msg;
          *msg.mutable_header() = this->command.header();
          msg.set_data(axis.velocity());
          this->velPublishers[name].Publish(msg);
          sentCommands.push_back(name + "[vel]");
        }
        else
          ignwarn << "Received velocity command for joint " << name << " which doesn't have a valid velocity target topic." << std::endl;
      }

      if (std::isfinite(axis.force()))
      {
        if (this->forcePublishers.find(name) != this->forcePublishers.end())
        {
          msgs::Double msg;
          *msg.mutable_header() = this->command.header();
          msg.set_data(axis.force());
          this->forcePublishers[name].Publish(msg);
          sentCommands.push_back(name + "[effort]");
        }
        else
          ignwarn << "Received force command for joint " << name << " which doesn't have a valid effort target topic." << std::endl;
      }

      if (!std::isfinite(axis.position()) && !std::isfinite(axis.velocity()) && !std::isfinite(axis.force()))
      {
        ignwarn << "Received command for joint " << name << " with no values to set." << std::endl;
      }
    }

    // clear the received command so that we don't repeat it
    this->command.clear_joint();

    // if (!sentCommands.empty())
    //   igndbg << "MultiJointCommandSystem has sent these commands: " << common::Join(sentCommands, ", ") << std::endl;
  }

  protected: void OnCmd(const msgs::Model &_msg)
  {
    if (!this->initialized)
      return;
    std::lock_guard<std::mutex> lock(this->commandLock);
    this->command = _msg;
  }

  public: void Reset(EntityComponentManager& _ecm)
  {
    std::lock_guard<std::mutex> lock(this->commandLock);
    this->command.clear_joint();
  }

  protected: bool initialized {false};
  protected: transport::Node node;
  protected: std::unordered_map<std::string, transport::Node::Publisher> posPublishers;
  protected: std::unordered_map<std::string, transport::Node::Publisher> velPublishers;
  protected: std::unordered_map<std::string, transport::Node::Publisher> forcePublishers;
  protected: Model model{kNullEntity};
  protected: msgs::Model command;
  protected: std::mutex commandLock;
};

}

IGNITION_ADD_PLUGIN(subt::MultiJointCommandSystem,
                    System,
                    ISystemConfigure,
                    ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(subt::MultiJointCommandSystem, "subt::MultiJointCommandSystem")