def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <plugin name="ignition::launch::GazeboFactory"
          filename="libignition-launch-gazebo-factory.so">
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{_z + 0.2} #{_roll} #{_pitch} #{_yaw}</pose>
    <world>#{_worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
    <include>
      <name>#{_name}</name>
      <uri>#{_modelURI}</uri>

      <!-- Track drive -->
      <plugin filename="libignition-gazebo-diff-drive-system.so"
              name="ignition::gazebo::systems::DiffDrive">
        <left_joint>left_1_wheel_joint</left_joint>
        <right_joint>right_1_wheel_joint</right_joint>
        <left_joint>left_2_wheel_joint</left_joint>
        <right_joint>right_2_wheel_joint</right_joint>
        <left_joint>left_3_wheel_joint</left_joint>
        <right_joint>right_3_wheel_joint</right_joint>
        <left_joint>left_4_wheel_joint</left_joint>
        <right_joint>right_4_wheel_joint</right_joint>
        <left_joint>left_5_wheel_joint</left_joint>
        <right_joint>right_5_wheel_joint</right_joint>
        <left_joint>left_6_wheel_joint</left_joint>
        <right_joint>right_6_wheel_joint</right_joint>
        <left_joint>left_7_wheel_joint</left_joint>
        <right_joint>right_7_wheel_joint</right_joint>
        <left_joint>left_8_wheel_joint</left_joint>
        <right_joint>right_8_wheel_joint</right_joint>
        <left_joint>left_9_wheel_joint</left_joint>
        <right_joint>right_9_wheel_joint</right_joint>
        <left_joint>left_10_wheel_joint</left_joint>
        <right_joint>right_10_wheel_joint</right_joint>
        <left_joint>left_11_wheel_joint</left_joint>
        <right_joint>right_11_wheel_joint</right_joint>
        <wheel_separation>0.5</wheel_separation>
        <wheel_radius>0.14</wheel_radius>
        <min_velocity>-1.2</min_velocity>
        <max_velocity>1.2</max_velocity>
        <min_acceleration>-1.0</min_acceleration>
        <max_acceleration>1.0</max_acceleration>
        <topic>/model/#{_name}/cmd_vel_relay</topic>
      </plugin>

      <!-- Joint state for lidar gimbal -->
      <plugin
        filename="libignition-gazebo-joint-controller-system.so"
        name="ignition::gazebo::systems::JointController">
        <joint_name>lidar_gimbal</joint_name>
      </plugin>
      <plugin
        filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
        <joint_name>lidar_gimbal</joint_name>
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
        <static_pose_publisher>true</static_pose_publisher>
        <static_pose_update_frequency>1</static_pose_update_frequency>
      </plugin>

      <!-- Battery plugin -->
      <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>57.6</voltage>
        <open_circuit_voltage_constant_coef>57.6</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-25.6</open_circuit_voltage_linear_coef>
        <initial_charge>50.0</initial_charge>
        <capacity>50.0</capacity>
        <resistance>0.00046875</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>15.00</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>

      <!-- Gas Sensor plugin -->"
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>

      <!-- Breadcrumbs -->
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
  </plugin>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
    <command>roslaunch --wait csiro_data61_ozbot_atr_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>q
    <command>roslaunch --wait csiro_data61_ozbot_atr_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=1</command>
  </executable>
  HEREDOC
end
