#include <memory>

#include <ignition/msgs/int32_v.pb.h>
#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/gazebo/components.hh>

using namespace ignition;
using namespace gazebo;

namespace subt
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

    if (!_sdf->HasElement("collision"))
    {
      ignerr << "LogicalContactSystem doesn't have any <collision> element. It will not do anything." << std::endl;
      return;
    }

    auto sdf = _sdf->Clone();
    for (auto collision = sdf->GetElement("collision"); collision; collision = collision->GetNextElement("collision"))
    {
      const auto name = collision->GetValue()->GetAsString();
      this->collisionNames.push_back(name);
      this->collisions[name] = kNullEntity;
    }

    std::string topic {"/model/" + model.Name(_ecm) + "/logical_contacts"};
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");

    this->pub = this->node.Advertise<ignition::msgs::Int32_V>(topic);

    std::stringstream ss;
    for (size_t i = 0; i < this->collisionNames.size(); ++i)
    {
      ss << "'" << this->collisionNames[i] << "'";
      if (i < this->collisionNames.size() - 1)
        ss << ", ";
    }
    ignmsg << "LogicalContactSystem publishing on [" << topic << "] is handling collisions [" << ss.str() << "]" << std::endl;
  }

  public: void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
  {
    ignition::msgs::Int32_V msg;
    size_t i = 0;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.mutable_data()->Resize(this->collisionNames.size(), -1);
    for (auto& name : this->collisionNames)
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
        const auto contacts = _ecm.Component<components::ContactSensorData>(entity);
        if (contacts != nullptr)
          msg.mutable_data()->Set(i, contacts->Data().contact_size() > 0);
      }

      i += 1;
    }

    this->pub.Publish(msg);
  }

  protected: transport::Node node;
  protected: transport::Node::Publisher pub;
  protected: std::vector<std::string> collisionNames;
  protected: std::unordered_map<std::string, Entity> collisions;
};

}

IGNITION_ADD_PLUGIN(subt::LogicalContactSystem,
                    System,
                    ISystemConfigure,
                    ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(subt::LogicalContactSystem, "subt::LogicalContactSystem")