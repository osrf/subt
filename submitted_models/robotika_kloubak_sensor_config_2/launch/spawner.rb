def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{_z + 0.25} #{_roll} #{_pitch} #{_yaw}</pose>
    <world>#{_worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
      <include>
        <name>#{_name}</name>
        <uri>#{_modelURI}</uri>
        <!-- Diff drive front -->
        <plugin filename=\"libignition-gazebo-diff-drive-system.so\"
                name=\"ignition::gazebo::systems::DiffDrive\">
          <left_joint>front_left_wheel_joint</left_joint>
          <right_joint>front_right_wheel_joint</right_joint>
          <wheel_separation>0.50</wheel_separation>
          <wheel_radius>0.15</wheel_radius>
          <topic>/model/#{_name}/cmd_vel_relay/front</topic>
          <odom_topic>/model/#{_name}/odometry/front</odom_topic>
          <min_velocity>-2.5</min_velocity>
          <max_velocity>2.5</max_velocity>
          <min_acceleration>-8.8</min_acceleration>
          <max_acceleration>8.8</max_acceleration>
        </plugin>
        <!-- Diff drive rear -->
        <plugin filename=\"libignition-gazebo-diff-drive-system.so\"
                name=\"ignition::gazebo::systems::DiffDrive\">
          <left_joint>rear_left_wheel_joint</left_joint>
          <right_joint>rear_right_wheel_joint</right_joint>
          <wheel_separation>0.50</wheel_separation>
          <wheel_radius>0.15</wheel_radius>
          <topic>/model/#{_name}/cmd_vel_relay/rear</topic>
          <odom_topic>/model/#{_name}/odometry/rear</odom_topic>
          <min_velocity>-2.5</min_velocity>
          <max_velocity>2.5</max_velocity>
          <min_acceleration>-8.8</min_acceleration>
          <max_acceleration>8.8</max_acceleration>
        </plugin>
        <!-- Publish robot state information -->
        <plugin filename=\"libignition-gazebo-pose-publisher-system.so\"
          name=\"ignition::gazebo::systems::PosePublisher\">
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
        <plugin filename=\"libignition-gazebo-linearbatteryplugin-system.so\"
          name=\"ignition::gazebo::systems::LinearBatteryPlugin\">
          <battery_name>linear_battery</battery_name>
          <voltage>25.2</voltage>
          <open_circuit_voltage_constant_coef>25.2</open_circuit_voltage_constant_coef>
          <open_circuit_voltage_linear_coef>-7.2</open_circuit_voltage_linear_coef>
          <initial_charge>13.214</initial_charge>
          <capacity>13.214</capacity>
          <resistance>0.052</resistance>
          <smooth_current_tau>5.0</smooth_current_tau>
          <power_load>3.33</power_load>
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
               <pose>-0.9 0 0.18 0 0 0</pose>
               <include>
                 <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Breadcrumb Node</uri>
              </include>
            </model>
          </sdf>
        </breadcrumb>
      </plugin>
        <!-- Wheel slip -->
        <plugin filename="libignition-gazebo-wheel-slip-system.so"
          name="ignition::gazebo::systems::WheelSlip">
          <wheel link_name="front_left_wheel">
            <slip_compliance_lateral>0.093</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>73.575</wheel_normal_force>
            <wheel_radius>0.147</wheel_radius>
          </wheel>
          <wheel link_name="rear_left_wheel">
            <slip_compliance_lateral>0.093</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>73.575</wheel_normal_force>
            <wheel_radius>0.147</wheel_radius>
          </wheel>
          <wheel link_name="front_right_wheel">
            <slip_compliance_lateral>0.093</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>73.575</wheel_normal_force>
            <wheel_radius>0.147</wheel_radius>
          </wheel>
          <wheel link_name="rear_right_wheel">
            <slip_compliance_lateral>0.093</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>73.575</wheel_normal_force>
            <wheel_radius>0.147</wheel_radius>
          </wheel>
        </plugin>
       </include>
      </sdf>
    </spawn>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
      <command>roslaunch --wait robotika_kloubak_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='kloubak_ros_ign_bridge'>
      <command>roslaunch --wait robotika_kloubak_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=1</command>
    </executable>
  HEREDOC
end
