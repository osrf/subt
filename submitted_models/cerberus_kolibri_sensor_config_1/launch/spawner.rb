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
        
        <!-- Quadrotor plugins -->
        <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
          name="ignition::gazebo::systems::MulticopterMotorModel">
          <robotNamespace>model/#{_name}</robotNamespace>
          <jointName>rotor_0_joint</jointName>
          <linkName>rotor_0</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>0.02246</motorConstant>
          <momentConstant>0.016754</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>0</motorNumber>
          <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
          <rollingMomentCoefficient>0</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>0.2</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        
        <!-- Controller plugin -->
        <plugin filename="libKolibriController.so" name="kolibri_controller::KolibriController">
        </plugin>
        
        <!-- Battery plugin -->
        <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
          name="ignition::gazebo::systems::LinearBatteryPlugin">
          <battery_name>linear_battery</battery_name>
          <voltage>22.2</voltage>
          <open_circuit_voltage_constant_coef>22.2</open_circuit_voltage_constant_coef>
          <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
          <initial_charge>49.0</initial_charge>
          <capacity>49.0</capacity>
          <resistance>0.061523</resistance>
          <smooth_current_tau>1.9499</smooth_current_tau>
          <power_load>58</power_load>
          <start_on_motion>true</start_on_motion>
        </plugin>
      
        <!-- Gas Detector plugin -->
        <plugin filename="libGasEmitterDetectorPlugin.so"
          name="subt::GasDetector">
          <topic>/model/#{_name}/gas_detected</topic>
          <update_rate>10</update_rate>
          <type>gas</type>
        </plugin>

      </include>
    </sdf>
  </spawn>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
    <executable name='kolibri_description'>
      <command>roslaunch --wait cerberus_kolibri_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='kolibri_ros_ign_bridge'>
      <command>roslaunch --wait cerberus_kolibri_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
