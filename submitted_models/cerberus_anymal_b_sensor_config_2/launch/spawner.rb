def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <spawn name='#{_name}'>
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
      <plugin filename="libAnymalBControlPlugin1.so"
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
        <!-- Noise parameters -->
        <linear_velocity_noise_mean>0 0 0.05</linear_velocity_noise_mean>
        <linear_velocity_noise_std_dev>0.05 0.05 0.00947</linear_velocity_noise_std_dev>
        <angular_velocity_noise_mean>0 0 0</angular_velocity_noise_mean>
        <angular_velocity_noise_std_dev>0.004 0.004 0.004</angular_velocity_noise_std_dev>
      </plugin>

      <!-- Publish robot state information -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_sensor_pose>true</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>1</static_update_frequency>
      </plugin>
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_sensor_pose>false</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>#{$enableGroundTruth}</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>false</static_publisher>
      </plugin>
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
      </plugin>

      <!-- Battery plugin -->
      <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>50</voltage>
        <open_circuit_voltage_constant_coef>50</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>12.3</initial_charge>
        <capacity>12.3</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>5.214</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>

      <!-- Gas Sensor plugin -->"
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
      <plugin filename="libignition-gazebo-breadcrumbs-system.so"
            name="ignition::gazebo::systems::Breadcrumbs">
        <topic>/model/#{_name}/breadcrumb/deploy</topic>
        <max_deployments>12</max_deployments>
        <disable_physics_time>3.0</disable_physics_time>
        <topic_statistics>true</topic_statistics>
        <breadcrumb>
          <sdf version="1.6">
            <model name="#{_name}__breadcrumb__">
              <pose>-0.45 0 0 0 0 0</pose>
              <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Breadcrumb Node</uri>
             </include>
           </model>
         </sdf>
       </breadcrumb>
     </plugin>
    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
    <command>roslaunch --wait cerberus_anymal_b_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait cerberus_anymal_b_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=1</command>
  </executable>
  HEREDOC
end
