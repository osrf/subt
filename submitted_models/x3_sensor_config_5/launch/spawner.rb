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
      <!-- Publish robot state information -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_sensor_pose>true</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>#{$enableGroundTruth}</publish_nested_model_pose>
      </plugin>
      <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
        name="ignition::gazebo::systems::MulticopterMotorModel">
        <robotNamespace>model/#{_name}</robotNamespace>
        <jointName>rotor_0_joint</jointName>
        <linkName>rotor_0</linkName>
        <turningDirection>ccw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>0</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
        name="ignition::gazebo::systems::MulticopterMotorModel">
        <robotNamespace>model/#{_name}</robotNamespace>
        <jointName>rotor_1_joint</jointName>
        <linkName>rotor_1</linkName>
        <turningDirection>ccw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>1</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/1</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
        name="ignition::gazebo::systems::MulticopterMotorModel">
        <robotNamespace>model/#{_name}</robotNamespace>
        <jointName>rotor_2_joint</jointName>
        <linkName>rotor_2</linkName>
        <turningDirection>cw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>2</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/2</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
        name="ignition::gazebo::systems::MulticopterMotorModel">
        <robotNamespace>model/#{_name}</robotNamespace>
        <jointName>rotor_3_joint</jointName>
        <linkName>rotor_3</linkName>
        <turningDirection>cw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>3</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <!-- Battery plugin -->
      <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>12.694</voltage>
        <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>18.0</initial_charge>
        <capacity>18.0</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>6.6</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
    </include>
    </sdf>
  </plugin>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='X3_controller'>
      <command>roslaunch --wait x3_control control.launch world_name:=#{_worldName} x3_name:=#{_name}</command>
    </executable>
    <executable name='x3_description'>
      <command>roslaunch --wait x3_sensor_config_5 description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='x3_ros_ign_bridge'>
      <command>roslaunch --wait x3_sensor_config_5 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} uav:=1 laser_scan:=0 top_scan:=1 bottom_scan:=1 rgbd_cam:=1    </command>
  </executable>
  HEREDOC
end
