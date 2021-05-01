#include <memory>

#include <ignition/msgs/int32_v.pb.h>
#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/gazebo/components.hh>

using namespace ignition;
using namespace gazebo;

namespace subt::lily
{

class LogicalContactSystem : public System, public ISystemConfigure, public ISystemPostUpdate
{
  public: void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         EntityComponentManager& _ecm, EventManager& _eventMgr) override
  {
    const Model model(_entity);
    if (!model.Valid(_ecm))
    {
      ignerr << "LogicalContactSystem should be attached to a model entity. Failed to initialize." << std::endl;
      return;
    }

    if (!_sdf->HasElement("group"))
    {
      ignerr << "LogicalContactSystem doesn't have any <group> element. It will not do anything." << std::endl;
      return;
    }

    auto sdf = _sdf->Clone();
    for (auto group = sdf->GetElement("group"); group; group = group->GetNextElement("group"))
    {
      if (!group->HasAttribute("name"))
      {
        ignerr << "LogicalContactSystem found a group without the 'name' attribute, ignoring it." << std::endl;
        continue;
      }

      const auto name = group->GetAttribute("name")->GetAsString();
      this->collisionNames[name].clear();  // initialize the map key

      for (auto collision = group->GetElement("collision"); collision; collision = collision->GetNextElement("collision"))
      {
        const auto collName = collision->GetValue()->GetAsString();
        if (collName.empty())
        {
          ignerr << "LogicalContactSystem found empty <collision> tag in group [" << name << "], it will be ignored." << std::endl;
          continue;
        }
        this->collisions[collName] = kNullEntity;
        this->collisionNames[name].push_back(collName);
      }
    }

    std::string topic {"/model/" + model.Name(_ecm) + "/logical_contacts"};
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");

    this->pub = this->node.Advertise<ignition::msgs::Int32_V>(topic);

    std::stringstream ss;
    for (const auto& groupPair : this->collisionNames)
    {
      ss << " - '" << groupPair.first << "' with collisions [";
      for (size_t i = 0;  i < groupPair.second.size(); ++i)
      {
        ss << "'" << groupPair.second[i] << "'";
        if (i < groupPair.second.size() - 1)
          ss << ", ";
      }
      ss << "]" << std::endl;
    }
    ignmsg << "LogicalContactSystem publishing on [" << topic << "] is handling the following groups:\n" << ss.str();
  }

  public: void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
  {
    ignition::msgs::Int32_V msg;
    size_t i = 0;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.mutable_data()->Resize(this->collisionNames.size(), -1);
    for (const auto& groupPair : this->collisionNames)
    {
      msg.mutable_data()->Set(i, false);
      for (const auto& name : groupPair.second)
      {
        auto& entity = this->collisions.at(name);
        if (entity == kNullEntity)
        {
          auto collisionEntities = _ecm.EntitiesByComponents(
            components::Collision(), components::Name(name));
          if (!collisionEntities.empty())
            entity = collisionEntities[0];
        }

        if (entity != kNullEntity)
        {
          const auto& contacts = _ecm.Component<components::ContactSensorData>(entity);
          if (contacts != nullptr && contacts->Data().contact_size() > 0)
          {
            msg.mutable_data()->Set(i,true);
            break;
          }
        }
      }

      i += 1;
    }

    this->pub.Publish(msg);
  }

  protected: transport::Node node;
  protected: transport::Node::Publisher pub;
  protected: std::map<std::string, std::vector<std::string>> collisionNames;
  protected: std::unordered_map<std::string, Entity> collisions;
};

}

IGNITION_ADD_PLUGIN(subt::lily::LogicalContactSystem,
                    System,
                    ISystemConfigure,
                    ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(subt::lily::LogicalContactSystem, "subt::lily::LogicalContactSystem")