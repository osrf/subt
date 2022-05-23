#include "kolibri_controller.hh"
#include <stdio.h>

using namespace ignition;
using namespace gazebo;
using namespace kolibri_controller;

math::Vector3d target_linear(0.,0.,0.);
math::Vector3d target_angular(0.,0.,0.);

void cmd_callback(const ignition::msgs::Twist &_msg){
  target_linear = math::Vector3d(_msg.linear().x(),_msg.linear().y(),_msg.linear().z());
  target_angular = math::Vector3d(_msg.angular().x(),_msg.angular().y(),_msg.angular().z());
}


KolibriController::KolibriController() {
}

KolibriController::~KolibriController() {
}

void KolibriController::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) {
  EntityComponentManager &_ecm_non_const = const_cast<EntityComponentManager&>(_ecm);
  
  if(_info.paused)
    return;

  // Since it's an OMAV we can use a simple approximation of the platform dynamics + 3rd party control system

  // Velocity constraints
  target_linear.Min(_max_vel);
  target_linear.Max(-_max_vel);
  target_angular.Min(_max_angular_vel);
  target_angular.Max(-_max_angular_vel);

  auto acc_linear = _K_prop*(target_linear - _linear_vel);
  auto acc_angular = _K_prop2*(target_angular - _angular_vel);
  
  // Acceleration constraints
  acc_linear.Min(_max_acc);
  acc_linear.Max(-_max_acc);
  acc_angular.Min(_max_angular_acc);
  acc_angular.Max(-_max_angular_acc);
    
  auto u_linear = (acc_linear+math::Vector3d(0.,0.,9.8))*_mass;  // It's an OMAV that does not roll or pitch, resulting in very simple motion.
  auto u_angular = acc_angular*math::Vector3d(_ixx,_iyy,_izz);
  //std::cout << " u_linear: " << u_linear << " u_angular: " << u_angular << "\n";
  this->base_link.AddWorldWrench(_ecm_non_const, u_linear, u_angular);
  
}

void KolibriController::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) {

  if(this->base_link.WorldPose(_ecm).has_value()){
    _linear_vel = this->base_link.WorldLinearVelocity(_ecm).value();
    _angular_vel = this->base_link.WorldAngularVelocity(_ecm).value();
  }
}

void KolibriController::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) {

  ignition::gazebo::Model model = Model(_entity);

  ignmsg << "\n\n\nPlugin loaded model: " << model.Name(_ecm) << "\n\n\n";

  this->base_link = ignition::gazebo::Link(model.LinkByName(_ecm,"base_link"));

  ignmsg<<base_link.Name(_ecm).value();

  if (!this->_nh.Subscribe("/model/kolibri/cmd_vel", cmd_callback)) {
    ignerr << "Error subscribing to topic [" << "/kolibri/cmd_vel" << "]" << std::endl;
  }

  if (_sdf->HasElement("mass")) _mass = _sdf->Get<double>("mass");
  if (_sdf->HasElement("ixx")) _ixx = _sdf->Get<double>("ixx");
  if (_sdf->HasElement("iyy")) _iyy = _sdf->Get<double>("iyy");
  if (_sdf->HasElement("izz")) _izz = _sdf->Get<double>("izz");
  _linear_vel = math::Vector3d(0,0,0);
  _angular_vel = math::Vector3d(0,0,0);
}

