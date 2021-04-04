def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
  <spawn name='#{_name}'>
      <name>#{_name}</name>
      <allow_renaming>false</allow_renaming>
      <pose>#{_x} #{_y} #{_z + 0.158} #{_roll} #{_pitch} #{_yaw}</pose>
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
        <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
        name="ignition::gazebo::systems::MulticopterMotorModel">
        <robotNamespace>model/#{_name}</robotNamespace>
        <jointName>rotor_0_joint</jointName>
        <linkName>rotor_0</linkName>
        <turningDirection>ccw</turningDirection>
        <timeConstantUp>0.0182</timeConstantUp>
        <timeConstantDown>0.0182</timeConstantDown>
        <maxRotVelocity>822.0</maxRotVelocity>
        <motorConstant>2.61217e-05</motorConstant>
        <momentConstant>0.016754</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>0</motorNumber>
        <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
        <rollingMomentCoefficient>0</rollingMomentCoefficient>
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
        <timeConstantUp>0.0182</timeConstantUp>
        <timeConstantDown>0.0182</timeConstantDown>
        <maxRotVelocity>822.0</maxRotVelocity>
        <motorConstant>2.61217e-05</motorConstant>
        <momentConstant>0.016754</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>1</motorNumber>
        <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
        <rollingMomentCoefficient>0</rollingMomentCoefficient>
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
        <timeConstantUp>0.0182</timeConstantUp>
        <timeConstantDown>0.0182</timeConstantDown>
        <maxRotVelocity>822.0</maxRotVelocity>
        <motorConstant>2.61217e-05</motorConstant>
        <momentConstant>0.016754</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>2</motorNumber>
        <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
        <rollingMomentCoefficient>0</rollingMomentCoefficient>
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
        <timeConstantUp>0.0182</timeConstantUp>
        <timeConstantDown>0.0182</timeConstantDown>
        <maxRotVelocity>822.0</maxRotVelocity>
        <motorConstant>2.61217e-05</motorConstant>
        <momentConstant>0.016754</momentConstant>
        <commandSubTopic>command/motor_speed</commandSubTopic>
        <motorNumber>3</motorNumber>
        <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
        <rollingMomentCoefficient>0</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <!-- MulticopterVelocityControl plugin -->
      <plugin
        filename="libMRSMultirotorController.so"
        name="ignition::gazebo::systems::MRSMultirotorController">
        <robotNamespace>model/#{_name}</robotNamespace>
        <commandSubTopic>cmd_vel</commandSubTopic>
        <motorControlPubTopic>command/motor_speed</motorControlPubTopic>
        <enableSubTopic>velocity_controller/enable</enableSubTopic>
        <comLinkName>base_link</comLinkName>
        <velocityGain>10 10 10</velocityGain>
        <attitudeGain>5 5 5</attitudeGain>
        <angularRateGain>2.0 2.0 3.0</angularRateGain>
        <maximumLinearAcceleration>7.49 8.13 7.29</maximumLinearAcceleration>
        <maximumLinearVelocity>8.6 8.6 4.47</maximumLinearVelocity>
        <maximumAngularVelocity>20 20 2.44</maximumAngularVelocity>
        <linearVelocityNoiseMean>0 0 0</linearVelocityNoiseMean>
        <linearVelocityNoiseStdDev>0.1105 0.1261 0.00947</linearVelocityNoiseStdDev>
        <angularVelocityNoiseMean>0 0 0</angularVelocityNoiseMean>
        <angularVelocityNoiseStdDev>0.004 0.004 0.004</angularVelocityNoiseStdDev>

          <rotorConfiguration>
            <rotor>
              <jointName>rotor_0_joint</jointName>
              <forceConstant>2.61217e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_1_joint</jointName>
              <forceConstant>2.61217e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_2_joint</jointName>
              <forceConstant>2.61217e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>-1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_3_joint</jointName>
              <forceConstant>2.61217e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>-1</direction>
            </rotor>
          </rotorConfiguration>
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
        <power_load>5.28</power_load>
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
  </spawn>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
    <executable name='robot_description'>
      <command>roslaunch --wait ctu_cras_norlab_x500_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='topics'>
      <command>roslaunch --wait ctu_cras_norlab_x500_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
