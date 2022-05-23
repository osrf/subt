//#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>


using namespace ignition;
using namespace gazebo;

namespace kolibri_controller
{
  class KolibriController:
    // This class is a system.
    public System,
    // This class also implements the ISystemPostUpdate interface.
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    public: KolibriController();
    public: ~KolibriController() override;
    public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override;
    public: void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm);
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;


    private:
      ignition::gazebo::Link base_link;
//      math::Vector3d tgt_vel=math::Vector3d(1,1,1);
//      math::Vector3d tgt_twist=math::Vector3d(1,1,1);
      math::Vector3d _linear_vel;
      math::Vector3d _angular_vel;

      ignition::transport::Node _nh;

      double _K_prop = 10.0;
      double _K_prop2 = 5.0;
      double _mass = 5.7, _ixx = 0.59, _iyy = 0.59, _izz = 0.95;
    
      // Velocity and acceleration constraints
      const math::Vector3d _max_vel = math::Vector3d(0.53, 0.46, 0.46);
      const math::Vector3d _max_angular_vel = math::Vector3d(0., 0., 1.06); // Platform cannot roll. Pitch was disabled in the SubT Systems track version due to odometry reliability issues.
      
      const math::Vector3d _max_acc = math::Vector3d(0.96, 1.07, 3.5);
      const math::Vector3d _max_angular_acc = math::Vector3d(0., 0., 0.5); 
    
  };
}

#include <ignition/plugin/Register.hh>
// Include a line in your source file for each interface implemented.
IGNITION_ADD_PLUGIN(
    kolibri_controller::KolibriController,
    System,
    kolibri_controller::KolibriController::ISystemConfigure,
    kolibri_controller::KolibriController::ISystemPreUpdate,
    kolibri_controller::KolibriController::ISystemPostUpdate)
