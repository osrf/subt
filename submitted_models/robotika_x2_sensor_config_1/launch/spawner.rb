def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  "<spawn name=\"#{_name}\">\n"\
  "  <name>#{_name}</name>\n"\
  "  <allow_renaming>false</allow_renaming>\n"\
  "  <pose>#{_x} #{_y} #{_z+0.063494} #{_roll} #{_pitch} #{_yaw}</pose>\n"\
  "  <world>#{$worldName}</world>\n"\
  "  <is_performer>true</is_performer>\n"\
  "  <sdf version='1.6'>\n"\
  "  <include>\n"\
  "    <name>#{_name}</name>\n"\
  "    <uri>#{_modelURI}</uri>\n"\
  "    <!-- Diff drive -->\n"\
  "    <plugin filename=\"libignition-gazebo-diff-drive-system.so\"\n"\
  "            name=\"ignition::gazebo::systems::DiffDrive\">\n"\
  "      <left_joint>front_left_wheel_joint</left_joint>\n"\
  "      <left_joint>rear_left_wheel_joint</left_joint>\n"\
  "      <right_joint>front_right_wheel_joint</right_joint>\n"\
  "      <right_joint>rear_right_wheel_joint</right_joint>\n"\
  "      <wheel_separation>#{0.33559 * 1.23}</wheel_separation>\n"\
  "      <wheel_radius>0.098</wheel_radius>\n"\
  "      <topic>/model/#{_name}/cmd_vel_relay</topic>\n"\
  "    </plugin>\n"\
  "    <!-- Publish robot state information -->\n"\
  "    <plugin filename=\"libignition-gazebo-pose-publisher-system.so\"\n"\
  "      name=\"ignition::gazebo::systems::PosePublisher\">\n"\
  "      <publish_link_pose>true</publish_link_pose>\n"\
  "      <publish_sensor_pose>true</publish_sensor_pose>\n"\
  "      <publish_collision_pose>false</publish_collision_pose>\n"\
  "      <publish_visual_pose>false</publish_visual_pose>\n"\
  "      <publish_nested_model_pose>#{$enableGroundTruth}</publish_nested_model_pose>\n"\
  "      <use_pose_vector_msg>true</use_pose_vector_msg>\n"\
  "      <static_publisher>true</static_publisher>\n"\
  "      <static_update_frequency>1</static_update_frequency>\n"\
  "    </plugin>\n"\
  "    <!-- Battery plugin -->\n"\
  "    <plugin filename=\"libignition-gazebo-linearbatteryplugin-system.so\"\n"\
  "      name=\"ignition::gazebo::systems::LinearBatteryPlugin\">\n"\
  "      <battery_name>linear_battery</battery_name>\n"\
  "      <voltage>12.694</voltage>\n"\
  "      <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>\n"\
  "      <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>\n"\
  "      <initial_charge>78.4</initial_charge>\n"\
  "      <capacity>78.4</capacity>\n"\
  "      <resistance>0.061523</resistance>\n"\
  "      <smooth_current_tau>1.9499</smooth_current_tau>\n"\
  "      <power_load>6.6</power_load>\n"\
  "      <start_on_motion>true</start_on_motion>\n"\
  "    </plugin>\n"\
  "    <!-- Gas Sensor plugin -->\n"\
  "    <plugin filename=\"libGasEmitterDetectorPlugin.so\"\n"\
  "      name=\"subt::GasDetector\">\n"\
  "      <topic>/model/#{_name}/gas_detected</topic>\n"\
  "      <update_rate>10</update_rate>\n"\
  "      <type>gas</type>\n"\
  "    </plugin>\n"\
  "  </include>\n"\
  "  </sdf>\n"\
  "</spawn>\n"\
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
    <command>roslaunch --wait robotika_x2_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait robotika_x2_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} rgbd_cam:=1 laser_scan:=1 stereo_cam:=0 lidar_3d:=0</command>
  </executable>
  HEREDOC
end
