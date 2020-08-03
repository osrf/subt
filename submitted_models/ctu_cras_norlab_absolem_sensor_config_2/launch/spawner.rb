def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{_z + 0.2} #{_roll} #{_pitch} #{_yaw}</pose>
    <world>#{_worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
    <include>
      <name>#{_name}</name>
      <uri>#{_modelURI}</uri>
      <!-- Diff drive -->
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>left_track_wheel1_j</left_joint>
        <left_joint>left_track_wheel2_j</left_joint>
        <left_joint>left_track_wheel3_j</left_joint>
        <left_joint>left_track_wheel4_j</left_joint>
        <left_joint>left_track_wheel5_j</left_joint>
        <left_joint>left_track_wheel6_j</left_joint>
        <left_joint>left_track_wheel7_j</left_joint>
        <left_joint>left_track_wheel8_j</left_joint>
        <left_joint>front_left_flipper_wheel1_j</left_joint>
        <left_joint>rear_left_flipper_wheel1_j</left_joint>
        <right_joint>right_track_wheel1_j</right_joint>
        <right_joint>right_track_wheel2_j</right_joint>
        <right_joint>right_track_wheel3_j</right_joint>
        <right_joint>right_track_wheel4_j</right_joint>
        <right_joint>right_track_wheel5_j</right_joint>
        <right_joint>right_track_wheel6_j</right_joint>
        <right_joint>right_track_wheel7_j</right_joint>
        <right_joint>right_track_wheel8_j</right_joint>
        <right_joint>front_right_flipper_wheel1_j</right_joint>
        <right_joint>rear_right_flipper_wheel1_j</right_joint>
        <wheel_separation>0.397</wheel_separation>
        <wheel_radius>0.089</wheel_radius>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
        <min_velocity>-0.4</min_velocity>
        <max_velocity>0.4</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>front_left_flipper_wheel2_j</left_joint>
        <left_joint>rear_left_flipper_wheel2_j</left_joint>
        <right_joint>front_right_flipper_wheel2_j</right_joint>
        <right_joint>rear_right_flipper_wheel2_j</right_joint>
        <wheel_separation>0.397</wheel_separation>
        <wheel_radius>0.074</wheel_radius>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
        <min_velocity>-0.4</min_velocity>
        <max_velocity>0.4</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>front_left_flipper_wheel3_j</left_joint>
        <left_joint>rear_left_flipper_wheel3_j</left_joint>
        <right_joint>front_right_flipper_wheel3_j</right_joint>
        <right_joint>rear_right_flipper_wheel3_j</right_joint>
        <wheel_separation>0.397</wheel_separation>
        <wheel_radius>0.059</wheel_radius>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
        <min_velocity>-0.4</min_velocity>
        <max_velocity>0.4</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>front_left_flipper_wheel4_j</left_joint>
        <left_joint>rear_left_flipper_wheel4_j</left_joint>
        <right_joint>front_right_flipper_wheel4_j</right_joint>
        <right_joint>rear_right_flipper_wheel4_j</right_joint>
        <wheel_separation>0.397</wheel_separation>
        <wheel_radius>0.044</wheel_radius>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
        <min_velocity>-0.4</min_velocity>
        <max_velocity>0.4</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>front_left_flipper_wheel5_j</left_joint>
        <left_joint>rear_left_flipper_wheel5_j</left_joint>
        <right_joint>front_right_flipper_wheel5_j</right_joint>
        <right_joint>rear_right_flipper_wheel5_j</right_joint>
        <wheel_separation>0.397</wheel_separation>
        <wheel_radius>0.029</wheel_radius>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
        <min_velocity>-0.4</min_velocity>
        <max_velocity>0.4</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <!-- Publish robot state information -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_sensor_pose>true</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>#{$enableGroundTruth}</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>1</static_update_frequency>
      </plugin>
      <!-- Battery plugin -->
      <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>12.694</voltage>
        <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>78.4</initial_charge>
        <capacity>78.4</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>4.95</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
      <!-- Gas Sensor plugin -->
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
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=1</command>
  </executable>
  HEREDOC
end
