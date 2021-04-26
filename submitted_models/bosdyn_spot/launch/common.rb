def _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins="", _pid=[250, 0.2, 5])
  joints = []
  legs = ["front_left", "front_right", "rear_left", "rear_right"]
  for leg in legs do
    for motor in ["hip_x", "hip_y", "knee"] do
      joints.append(leg + "_" + motor)
    end
  end

  actuators = ""
  state_publisher_joints = ""
  multi_joint_command_joints = ""

  # the PID values for the position controller system are taken from config/ros_control/ros_control.yaml
  # and are used just to keep the robot standing until the user starts commanding it from the
  # solution container; the PID gains here were verified to be able to keep the robot standing (just the bare robot);
  # as soon as the user starts the solution code, the PID gains get reconfigured
  # to match whatever is set by the solution code (they can even be changed dynamically)
  for joint in joints do
    actuator=<<-HEREDOC
    <plugin filename="libignition-gazebo-apply-joint-force-system.so"
        name="ignition::gazebo::systems::ApplyJointForce">
      <joint_name>#{joint}</joint_name>
    </plugin>
    <plugin filename="configurable-joint-position-controller-system"
        name="subt::ConfigurableJointPositionController">
      <joint_name>#{joint}</joint_name>
      <keep_initial_pos />
      <p_gain>#{_pid[0]}</p_gain>
      <i_gain>#{_pid[1]}</i_gain>
      <d_gain>#{_pid[2]}</d_gain>
    </plugin>
    HEREDOC
    actuators += actuator

    state_publisher_joints += "<joint_name>" + joint + "</joint_name>\n"

    multi_joint_command_joints += "<joint name=\"" + joint + "\" />\n"
  end

  logical_contact_collisions = ""

  for leg in legs do
    logical_contact_collisions += "<group name='" + leg + "'>\n";
    logical_contact_collisions += "  <collision>" + leg + "_lower_leg_collision</collision>\n";
    logical_contact_collisions += "  <collision>" + leg + "_lower_leg_collision_1</collision>\n";
    logical_contact_collisions += "</group>\n";
  end

  <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{_z + 0.7} #{_roll} #{_pitch} #{_yaw}</pose>
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
      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher">
        #{state_publisher_joints}
      </plugin>
      #{actuators}
      <plugin filename="libmulti-joint-command-system.so" name="subt::MultiJointCommandSystem">
        #{multi_joint_command_joints}
      </plugin>
      <plugin filename="logical-contact-system" name="subt::LogicalContactSystem">
        #{logical_contact_collisions}
        <topic>/model/#{_name}/leg_collisions</topic>
      </plugin>
      #{_additionalSpawnPlugins}
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
    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def _rosExecutables(_name, _worldName)
  # description.launch is included from vehicle_topics.launch
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait --screen bosdyn_spot vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
