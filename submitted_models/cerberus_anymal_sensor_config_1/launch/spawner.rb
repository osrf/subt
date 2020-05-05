def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <plugin name="ignition::launch::GazeboFactory"
          filename="libignition-launch-gazebo-factory.so">
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{_z + 1.2} #{_roll} #{_pitch} #{_yaw}</pose>
    <world>#{_worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
    <include>
      <name>#{_name}</name>
      <uri>#{_modelURI}</uri>
      <!-- ANYmal control -->
      <plugin filename="libAnymalControlPlugin1.so"
              name="ignition::gazebo::systems::AnymalControlPlugin">
        <jointNames>
          <LF_HAA>LF_HAA</LF_HAA>
          <LF_HFE>LF_HFE</LF_HFE>
          <LF_KFE>LF_KFE</LF_KFE>
          <RF_HAA>RF_HAA</RF_HAA>
          <RF_HFE>RF_HFE</RF_HFE>
          <RF_KFE>RF_KFE</RF_KFE>
          <LH_HAA>LH_HAA</LH_HAA>
          <LH_HFE>LH_HFE</LH_HFE>
          <LH_KFE>LH_KFE</LH_KFE>
          <RH_HAA>RH_HAA</RH_HAA>
          <RH_HFE>RH_HFE</RH_HFE>
          <RH_KFE>RH_KFE</RH_KFE>
        </jointNames>
        <desired_joint_positions_topic>/model/#{_name}/desired_joint_positions_relay</desired_joint_positions_topic>
        <robot_base_twist_topic>/model/#{_name}/robot_base_twist</robot_base_twist_topic>
        <controller_update_frequency>500.0</controller_update_frequency>
      </plugin>
      <!-- Publish robot state information -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_sensor_pose>true</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>#{$enableGroundTruth}</publish_nested_model_pose>
      </plugin>

      <plugin filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
      </plugin>
      <!-- Battery plugin -->
      <!-- WARNING: this plugin seems to have a bug and freezes the joints even though
           the battery is not empty. While investigating this issue, we need to comment the plugin
           otherwise we are not able to set joint torques because the joints are frozen.
           See opened issue: https://bitbucket.org/ignitionrobotics/ign-gazebo/issues/55/battery-plugin-freezes-joints
      -->
      <!--plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>12.694</voltage>
        <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>78.4</initial_charge>
        <capacity>78.4</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>6.6</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin-->
    </include>
    </sdf>
  </plugin>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
    <command>roslaunch --wait cerberus_anymal_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait cerberus_anymal_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
