def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
    _config=8
    uav=1
    laserScan=0
    stereoCam=0
    rgbdCam=0
    if _config == "2"
      rgbdCam=1
    end
    if _config == "3" or _config == "4"
      laserScan=1
    end
    if _config == "5"
      stereoCam=1
    if _config == "6"
      laserScan=1
      topScan=1
      bottomScan=1
      rgbdCam=1
    end
    if _config == "8"
      laserScan=1
      topScan=1
      bottomScan=1
      rgbdCam=1
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
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
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
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
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
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
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
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
          <momentConstant>0.016754</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>3</motorNumber>
          <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
          <rollingMomentCoefficient>0</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
          name="ignition::gazebo::systems::MulticopterMotorModel">
          <robotNamespace>model/#{_name}</robotNamespace>
          <jointName>rotor_4_joint</jointName>
          <linkName>rotor_4</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
          <momentConstant>0.016754</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>4</motorNumber>
          <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
          <rollingMomentCoefficient>0</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/4</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin filename="libignition-gazebo-multicopter-motor-model-system.so"
          name="ignition::gazebo::systems::MulticopterMotorModel">
          <robotNamespace>model/#{_name}</robotNamespace>
          <jointName>rotor_5_joint</jointName>
          <linkName>rotor_5</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0182</timeConstantUp>
          <timeConstantDown>0.0182</timeConstantDown>
          <maxRotVelocity>1000.0</maxRotVelocity>
          <motorConstant>1.269e-05</motorConstant>
          <momentConstant>0.016754</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>5</motorNumber>
          <rotorDragCoefficient>2.0673e-04</rotorDragCoefficient>
          <rollingMomentCoefficient>0</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/5</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin
          filename="libignition-gazebo-multicopter-control-system.so"
          name="ignition::gazebo::systems::MulticopterVelocityControl">
          <robotNamespace>model/#{_name}</robotNamespace>
          <commandSubTopic>cmd_vel</commandSubTopic>
          <motorControlPubTopic>command/motor_speed</motorControlPubTopic>
          <enableSubTopic>velocity_controller/enable</enableSubTopic>
          <comLinkName>base_link</comLinkName>
          <velocityGain>6 6 10</velocityGain>
          <attitudeGain>4 4 2</attitudeGain>
          <angularRateGain>0.7 0.7 0.7</angularRateGain>
          <maximumLinearAcceleration>1 1 2</maximumLinearAcceleration>
          <maximumLinearVelocity>5 5 5</maximumLinearVelocity>
          <maximumAngularVelocity>3 3 3</maximumAngularVelocity>
          <linearVelocityNoiseMean>0 0 0</linearVelocityNoiseMean>
          <!-- linearVelocityNoiseStdDev based on error values reported in the paper Shen et. al., -->
          <!-- Vision-Based State Estimation and Trajectory Control Towards High-Speed Flight with a Quadrotor -->
          <!-- http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.490.7958&rep=rep1&type=pdf -->
          <linearVelocityNoiseStdDev>0.1105 0.1261 0.0947</linearVelocityNoiseStdDev>
          <angularVelocityNoiseMean>0 0 0</angularVelocityNoiseMean>
          <!-- angularVelocityNoiseStdDev values based on ADIS16448's Rate Noise Density with a sample  -->
          <!-- time of 0.004 ms. -->
          <angularVelocityNoiseStdDev>0.004 0.004 0.004</angularVelocityNoiseStdDev>

          <rotorConfiguration>
            <rotor>
              <jointName>rotor_0_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_1_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>-1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_2_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_3_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>-1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_4_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
              <momentConstant>1.6754e-2</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>rotor_5_joint</jointName>
              <forceConstant>1.269e-05</forceConstant>
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
          <power_load>6.6</power_load>
          <start_on_motion>true</start_on_motion>
        </plugin>
       <!-- Gas Sensor plugin -->"
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
  <executable name='X4_controller'>
      <command>roslaunch --wait x4_control control.launch world_name:=#{_worldName} x4_name:=#{_name}</command>
    </executable>
    <executable name='x4_description'>
      <command>roslaunch --wait ssci_x4_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='x4_ros_ign_bridge'>
      <command>roslaunch --wait ssci_x4_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} uav:=1 lidar_3d:=1 laser_scan:=0 top_scan:=1 bottom_scan:=1 stereo_cam:=0 rgbd_cam:=0    </command>
  </executable>
  HEREDOC
end
