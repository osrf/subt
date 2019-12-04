def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  <<-HEREDOC
    _config=6
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
    <plugin name="ignition::launch::GazeboFactory"
            filename="libignition-launch-gazebo-factory.so">
      <name>#{_name}</name>
      <allow_renaming>false</allow_renaming>
      <pose>#{_x} #{_y} 0.2 0 0 0</pose>
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
      <command>roslaunch --wait subt_ros x4_description.launch world_name:=#{_worldName} name:=#{_name}</command>
    </executable>
    <executable name='x4_ros_ign_bridge'>
      <command>roslaunch --wait ssci_x4_sensor_config_8 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} uav:=1 lidar_3d:=1 laser_scan:=0 top_scan:=0 bottom_scan:=0 stereo_cam:=0 rgbd_cam:=0    </command>
  </executable>
  HEREDOC
end
