def _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF="")

  # PID default values
  # TODO modify this
  pid=[250, 0.2, 5]

  # all joint names
  joints = ""
  # leg names
  legs = ["1", "2", "3", "4", "5", "6"]
  for leg in legs do
    for motor in ["J1_base", "J2_shoulder", "J3_elbow"] do
      # this string should fit joint names in xacro, I guess
      joints += "<joint_name>L" + leg + "_" + motor + "</joint_name>\n"
    end
  end

  # creating HEREDOC for contact collisions detection
  logical_contact_collisions = ""

  for leg in legs do
    logical_contact_collisions += "<group name='" + leg + "'>\n";
    logical_contact_collisions += "  <collision>elbow#{leg}_foot/INPUT_INTERFACE_collision</collision>\n";
    logical_contact_collisions += "  <collision>elbow#{leg}_foot/INPUT_INTERFACE_collision_1</collision>\n";
    logical_contact_collisions += "  <collision>elbow#{leg}_foot/INPUT_INTERFACE_collision_2</collision>\n";
    logical_contact_collisions += "  <collision>elbow#{leg}_foot/INPUT_INTERFACE_collision_3</collision>\n";
    logical_contact_collisions += "  <collision>elbow#{leg}_foot/INPUT_INTERFACE_collision_4</collision>\n";
    logical_contact_collisions += "</group>\n";
  end


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
        <power_load>9.9</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>

      <!-- joint position publisher -->
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher">
        #{joints}
      </plugin>

      <!-- logical contact system -->
      <plugin filename="lily-logical-contact-system" name="subt::lily::LogicalContactSystem">
        #{logical_contact_collisions}
        <topic>/model/#{_name}/leg_collisions</topic>
      </plugin>
      #{_additionalSDF}
    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def _rosExecutables(_name, _worldName, _runController=0)
  <<-HEREDOC
  <executable name='topics'>
    <!-- [!] package name here -->
    <command>roslaunch --wait --screen ctu_cras_norlab_lily_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} run_controller:=#{_runController}</command>
  </executable>
  HEREDOC
end