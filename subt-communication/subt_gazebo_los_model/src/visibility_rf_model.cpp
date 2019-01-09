#include <subt_gazebo_los_model/visibility_rf_model.h>

#include <gazebo/physics/physics.hh>

namespace subt
{

namespace rf_interface
{

namespace visibility_model
{

VisibilityModel::VisibilityModel()
{
  world = gazebo::physics::get_world();
      
  if (!this->visibilityTable.Load()) {
    ROS_ERROR("Unable to load visibility table data files");
    return;
  }

  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::HOST);
  this->node.Advertise("/subt/comms_model/visualize",
                       &VisibilityModel::VisualizeVisibility, this, opts);
}

rf_power VisibilityModel::compute_received_power(const double& tx_power,
                                                 radio_state& tx_state,
                                                 radio_state& rx_state)
{
  // Use this->visibilityTable.Cost(tx_state, rx_state) to compute
  // pathloss and thus, received power

  return {tx_power, 0.0};
}

bool VisibilityModel::VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                                          ignition::msgs::Boolean &_rep)
{
  _rep.set_data(false);

  std::map<int, ignition::msgs::Marker> per_cost_markers;

  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  markerMsg.mutable_lifetime()->set_sec(0.0);
  markerMsg.mutable_lifetime()->set_nsec(0.0);

  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  // Available colors
  //
  // RedGlow, YellowGlow, GreenGlow, TurquoiseGlow, BlueGlow
  // High (good)                                    Low (bad)

  std::map<int, std::string> index_to_color;
  index_to_color[0] = "Gazebo/RedGlow";
  index_to_color[1] = "Gazebo/YellowGlow";
  index_to_color[2] = "Gazebo/GreenGlow";
  index_to_color[3] = "Gazebo/TurquoiseGlow";
  index_to_color[4] = "Gazebo/BlueGlow";

  for(int i=0; i < 5; ++i) {
    markerMsg.set_id(i);
    
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    matMsg->mutable_script()->set_name(index_to_color[i]);
    ignition::msgs::Set(markerMsg.mutable_scale(),
                        ignition::math::Vector3d(1.0, 1.0, 1.0));

    per_cost_markers.insert(std::make_pair(i, markerMsg));
  }
  
  std::string modelName = _req.data();
  auto model = this->world->ModelByName(modelName);
  if (!model)
  {
    ignerr << "[" << modelName << "] model not found" << std::endl;
    return true;
  }

  ignition::math::Vector3d from = model->WorldPose().Pos();
  

  uint64_t index = 0;
  for (auto const &entry : this->visibilityTable.Vertices())
  {
    const auto &toTuple = entry.first;
    ignition::math::Vector3d to = ignition::math::Vector3d(
      std::get<0>(toTuple), std::get<1>(toTuple), std::get<2>(toTuple));
    double cost = this->visibilityTable.Cost(from, to);
    if (cost <= this->commsCostMax)
    {
      auto m = per_cost_markers.find((int)( ((commsCostMax+1.0)/5.0)*cost/10.0) );
      if(m == per_cost_markers.end()) {
        ROS_WARN("Have not pre-allocated a marker for cost: %f (%d)",
                 cost,
                 (int)( ((commsCostMax+1.0)/5.0)*cost/10.0) );
        continue;
        //per_cost_markers.insert(std::make_pair(round(cost), ignition::msgs::Marker()));
      }

      // markerMsg.set_id(index++);
      // ignition::msgs::Set(markerMsg.mutable_pose(),
      //              ignition::math::Pose3d(to.X(), to.Y(), to.Z(), 0, 0, 0));
      ignition::msgs::Set(m->second.add_point(),
                          ignition::math::Vector3d(to.X(), to.Y(), to.Z()));
    }
  }

  this->node.Request("/marker", per_cost_markers[0]);
  this->node.Request("/marker", per_cost_markers[1]);
  this->node.Request("/marker", per_cost_markers[2]);
  this->node.Request("/marker", per_cost_markers[3]);
  this->node.Request("/marker", per_cost_markers[4]);

  _rep.set_data(true);
  return true;
}


}
}
}
