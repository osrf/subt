#ifndef CTU_CRAS_NORLAB_ABSOLEM_SENSOR_CONFIG_1_ECM_HELPER_HH
#define CTU_CRAS_NORLAB_ABSOLEM_SENSOR_CONFIG_1_ECM_HELPER_HH

#include <ignition/gazebo/EntityComponentManager.hh>

namespace cras
{

// This is a helper meant to be upstreamed to ign-gazebo
// TODO(peci1) remove this once https://github.com/ignitionrobotics/ign-gazebo/pull/629 is merged
template<typename ComponentTypeT>
ComponentTypeT* ComponentDefault(
  ignition::gazebo::EntityComponentManager& _ecm,
  const ignition::gazebo::Entity _entity,
  const typename ComponentTypeT::Type& _default = typename ComponentTypeT::Type())
{
  auto comp = _ecm.Component<ComponentTypeT>(_entity);
  if (!comp)
  {
    _ecm.CreateComponent(_entity, ComponentTypeT(_default));
    comp = _ecm.Component<ComponentTypeT>(_entity);
  }
  return comp;
}

}

#endif //CTU_CRAS_NORLAB_ABSOLEM_SENSOR_CONFIG_1_ECM_HELPER_HH
