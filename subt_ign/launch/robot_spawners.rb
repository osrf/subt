# This is a common file intended to be included in all competition world
# launchers that want to spawn robots.

# Library of functions for robots
require File.dirname(__FILE__) + "/robot_common_defs.rb"

# Needed for adding marsupial robots
require "rexml/document"

def spawnX1(_name, _config, _x = 0.0, _y = 0.0, _z = 0.0, _yaw = 0.0)
  uav=0
  laserScan=0
  stereoCam=0
  lidar3d=0
  if _config == "1" or _config == "2"
    laserScan=1
  end
  if _config == "3" or _config == "4" or _config == "7" or _config == "8"
    lidar3d=1
  end
  if _config == "5"
    stereoCam=1
  end

  max_breadcrumbs = 0
  if _config == "7" or _config == "8"
    max_breadcrumbs = 12
  end

  breadcrumb_drop_offset = '-0.45 0 0 0 0 0'

  z = _z + 0.2

  robot = Robot.new(_name, "X1_SENSOR_CONFIG_" + _config)

  spawn = <<-HEREDOC
    <spawn name='#{_name}'>
      <name>#{_name}</name>
      <allow_renaming>false</allow_renaming>
      <pose>#{_x} #{_y} #{z} 0 0 #{_yaw}</pose>
      <world>#{$worldName}</world>
      <is_performer>true</is_performer>
      <sdf version='1.6'>
      <include>
        <name>#{_name}</name>
        <uri>#{robot.modelURI()}</uri>
        <!-- Diff drive -->
        <plugin filename="libignition-gazebo-diff-drive-system.so"
                name="ignition::gazebo::systems::DiffDrive">
          <left_joint>front_left_wheel_joint</left_joint>
          <left_joint>rear_left_wheel_joint</left_joint>
          <right_joint>front_right_wheel_joint</right_joint>
          <right_joint>rear_right_wheel_joint</right_joint>
          <wheel_separation>#{0.45649 * 1.5}</wheel_separation>
          <wheel_radius>0.1651</wheel_radius>
          <topic>/model/#{_name}/cmd_vel_relay</topic>
          <min_velocity>-1</min_velocity>
          <max_velocity>1</max_velocity>
          <min_acceleration>-3</min_acceleration>
          <max_acceleration>3</max_acceleration>
        </plugin>

        <!-- Plugins common to all robots -->
        #{robot.commonPlugins($enableGroundTruth)}

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
          <power_load>6.6</power_load>
          <start_on_motion>true</start_on_motion>
        </plugin>

        <!-- Breadcrumbs -->
        #{robot.breadcrumbsPlugins(max_breadcrumbs, breadcrumb_drop_offset)}

        <!-- Wheel plugin -->
        <plugin filename="libignition-gazebo-wheel-slip-system.so"
          name="ignition::gazebo::systems::WheelSlip">
          <wheel link_name="front_left_wheel">
            <slip_compliance_lateral>0.172</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>138.767</wheel_normal_force>
            <wheel_radius>0.1651</wheel_radius>
          </wheel>
          <wheel link_name="rear_left_wheel">
            <slip_compliance_lateral>0.172</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>138.767</wheel_normal_force>
            <wheel_radius>0.1651</wheel_radius>
          </wheel>
          <wheel link_name="front_right_wheel">
            <slip_compliance_lateral>0.172</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>138.767</wheel_normal_force>
            <wheel_radius>0.1651</wheel_radius>
          </wheel>
          <wheel link_name="rear_right_wheel">
            <slip_compliance_lateral>0.172</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>138.767</wheel_normal_force>
            <wheel_radius>0.1651</wheel_radius>
          </wheel>
        </plugin>
      </include>
      </sdf>
    </spawn>
  HEREDOC

  exec = <<-HEREDOC
    <executable name='x1_description'>
      <command>roslaunch --wait subt_ros x1_description.launch world_name:=#{$worldName} name:=#{_name}</command>
    </executable>
    <executable name='x1_ros_ign_bridge'>
      <command>roslaunch --wait subt_ros vehicle_topics.launch world_name:=#{$worldName} name:=#{_name} uav:=#{uav} laser_scan:=#{laserScan} stereo_cam:=#{stereoCam} lidar_3d:=#{lidar3d} breadcrumbs:=#{max_breadcrumbs>0?1:0}</command>
    </executable>
  HEREDOC

  return spawn, exec
end

def spawnX2(_name, _config, _x = 0.0, _y = 0.0, _z = 0.0, _yaw = 0.0)
  uav=0
  laserScan=0
  stereoCam=0
  lidar3d=0
  if _config == "1" or _config == "2" or _config == "3" or _config == "4" or _config == "9"
    laserScan=1
  end
  if _config == "5"
    stereoCam=1
  end
  if _config == "6" or _config == "8"
    lidar3d=1
  end

  max_breadcrumbs = 0
  if _config == "8" or _config == "9"
    max_breadcrumbs = 6
  end

  breadcrumb_drop_offset = '-0.24 0 0 0 0 0'

  z = _z + 0.063494
  
  robot = Robot.new(_name, "X2_SENSOR_CONFIG_" + _config)

  spawn = <<-HEREDOC
    <spawn name='#{_name}'>
      <name>#{_name}</name>
      <allow_renaming>false</allow_renaming>
      <pose>#{_x} #{_y} #{z} 0 0 #{_yaw}</pose>
      <world>#{$worldName}</world>
      <is_performer>true</is_performer>
      <sdf version='1.6'>
      <include>
        <name>#{_name}</name>
        <uri>#{robot.modelURI()}</uri>
        <!-- Diff drive -->
        <plugin filename="libignition-gazebo-diff-drive-system.so"
                name="ignition::gazebo::systems::DiffDrive">
          <left_joint>front_left_wheel_joint</left_joint>
          <left_joint>rear_left_wheel_joint</left_joint>
          <right_joint>front_right_wheel_joint</right_joint>
          <right_joint>rear_right_wheel_joint</right_joint>
          <wheel_separation>#{0.33559 * 1.23}</wheel_separation>
          <wheel_radius>0.098</wheel_radius>
          <topic>/model/#{_name}/cmd_vel_relay</topic>
          <min_velocity>-2</min_velocity>
          <max_velocity>2</max_velocity>
          <min_acceleration>-6</min_acceleration>
          <max_acceleration>6</max_acceleration>
        </plugin>

        <!-- Plugins common to all robots -->
        #{robot.commonPlugins($enableGroundTruth)}

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
          <power_load>6.6</power_load>
          <start_on_motion>true</start_on_motion>
        </plugin>

        <!-- Breadcrumbs -->
        #{robot.breadcrumbsPlugins(max_breadcrumbs, breadcrumb_drop_offset)}

        <!-- Wheel plugin -->
        <plugin filename="libignition-gazebo-wheel-slip-system.so"
          name="ignition::gazebo::systems::WheelSlip">
          <wheel link_name="front_left_wheel">
            <slip_compliance_lateral>0.116</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>45.20448</wheel_normal_force>
            <wheel_radius>0.098</wheel_radius>
          </wheel>
          <wheel link_name="rear_left_wheel">
            <slip_compliance_lateral>0.116</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>45.20448</wheel_normal_force>
            <wheel_radius>0.098</wheel_radius>
          </wheel>
          <wheel link_name="front_right_wheel">
            <slip_compliance_lateral>0.116</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>45.20448</wheel_normal_force>
            <wheel_radius>0.098</wheel_radius>
          </wheel>
          <wheel link_name="rear_right_wheel">
            <slip_compliance_lateral>0.116</slip_compliance_lateral>
            <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
            <wheel_normal_force>45.20448</wheel_normal_force>
            <wheel_radius>0.098</wheel_radius>
          </wheel>
        </plugin>
      </include>
      </sdf>
    </spawn>
  HEREDOC

  exec = <<-HEREDOC
    <executable name='x2_description'>
      <command>roslaunch --wait subt_ros x2_description.launch world_name:=#{$worldName} name:=#{_name}</command>
    </executable>
    <executable name='x2_ros_ign_bridge'>
      <command>roslaunch --wait subt_ros vehicle_topics.launch world_name:=#{$worldName} name:=#{_name} uav:=#{uav} laser_scan:=#{laserScan} stereo_cam:=#{stereoCam} lidar_3d:=#{lidar3d} breadcrumbs:=#{max_breadcrumbs>0?1:0}</command>
    </executable>
  HEREDOC

  return spawn, exec
end

def spawnX3(_name, _config, _x = 0.0, _y = 0.0, _z = 0.0, _yaw = 0.0)
  uav=1
  laserScan=0
  stereoCam=0
  rgbdCam=0
  if _config == "3" || _config == "4"
    rgbdCam=1
  end
  if _config == "5"
    stereoCam=1
  end

  z = _z + 0.053302

  robot = Robot.new(_name, "X3_SENSOR_CONFIG_" + _config)

  spawn = <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{z} 0 0 #{_yaw}</pose>
    <world>#{$worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
    <include>
      <name>#{_name}</name>
      <uri>#{robot.modelURI()}</uri>
      <!-- Plugins common to all robots -->
      #{robot.commonPlugins($enableGroundTruth)}

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
      <!--Multicopter velocity controller-->
      <plugin
        filename="libignition-gazebo-multicopter-control-system.so"
        name="ignition::gazebo::systems::MulticopterVelocityControl">
        <robotNamespace>model/#{_name}</robotNamespace>
        <commandSubTopic>cmd_vel</commandSubTopic>
        <motorControlPubTopic>command/motor_speed</motorControlPubTopic>
        <enableSubTopic>velocity_controller/enable</enableSubTopic>
        <comLinkName>base_link</comLinkName>
        <velocityGain>2.7 2.7 2.7</velocityGain>
        <attitudeGain>2 3 0.15</attitudeGain>
        <angularRateGain>0.4 0.52 0.18</angularRateGain>
        <maximumLinearAcceleration>1 1 2</maximumLinearAcceleration>
        <maximumLinearVelocity>5 5 5</maximumLinearVelocity>
        <maximumAngularVelocity>3 3 3</maximumAngularVelocity>
        <linearVelocityNoiseMean>0 0 0.05</linearVelocityNoiseMean>
        <!-- linearVelocityNoiseStdDev based on error values reported in the paper Shen et. al., -->
        <!-- Vision-Based State Estimation and Trajectory Control Towards High-Speed Flight with a Quadrotor -->
        <!-- http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.490.7958&rep=rep1&type=pdf -->
        <linearVelocityNoiseStdDev>0.1105 0.1261 0.00947</linearVelocityNoiseStdDev>
        <angularVelocityNoiseMean>0 0 0</angularVelocityNoiseMean>
        <!-- angularVelocityNoiseStdDev values based on ADIS16448's Rate Noise Density with a sample  -->
        <!-- time of 0.004 ms. -->
        <angularVelocityNoiseStdDev>0.004 0.004 0.004</angularVelocityNoiseStdDev>
        <rotorConfiguration>
          <rotor>
            <jointName>rotor_0_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_1_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_2_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>-1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_3_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
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
    </include>
    </sdf>
  </spawn>
  HEREDOC

  exec = <<-HEREDOC
    <executable name='x3_description'>
      <command>roslaunch --wait subt_ros x3_description.launch world_name:=#{$worldName} name:=#{_name}</command>
    </executable>
    <executable name='x3_ros_ign_bridge'>
      <command>roslaunch --wait subt_ros vehicle_topics.launch world_name:=#{$worldName} name:=#{_name} uav:=#{uav} laser_scan:=#{laserScan} stereo_cam:=#{stereoCam} rgbd_cam:=#{rgbdCam} breadcrumbs:=0</command>
    </executable>
  HEREDOC

  return spawn, exec
end

def spawnX4(_name, _config, _x = 0.0, _y = 0.0, _z = 0.0, _yaw = 0.0)
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
  end

  z = _z + 0.125

  robot = Robot.new(_name, "X4_SENSOR_CONFIG_" + _config)

  spawn = <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <pose>#{_x} #{_y} #{z} 0 0 #{_yaw}</pose>
    <world>#{$worldName}</world>
    <is_performer>true</is_performer>
    <sdf version='1.6'>
    <include>
      <name>#{_name}</name>
      <uri>#{robot.modelURI()}</uri>

      <!-- Plugins common to all robots -->
      #{robot.commonPlugins($enableGroundTruth)}

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
      <!--Multicopter velocity controller-->
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
      </include>
    </sdf>
  </spawn>
  HEREDOC

  exec = <<-HEREDOC
  <executable name='x4_description'>
    <command>roslaunch --wait subt_ros x4_description.launch world_name:=#{$worldName} name:=#{_name}</command>
  </executable>
  <executable name='x4_ros_ign_bridge'>
    <command>roslaunch --wait subt_ros vehicle_topics.launch world_name:=#{$worldName} name:=#{_name} uav:=#{uav} laser_scan:=#{laserScan} stereo_cam:=#{stereoCam} rgbd_cam:=#{rgbdCam} breadcrumbs:=0</command>
  </executable>
  HEREDOC

  return spawn, exec
end

def spawnTeamBase(_name, _x, _y, _z, _yaw)
  spawn = <<-HEREDOC
  <spawn name='#{_name}'>
    <name>#{_name}</name>
    <allow_renaming>false</allow_renaming>
    <world>#{$worldName}</world>
    <is_performer>false</is_performer>
    <sdf version='1.6'>
    <model name='#{_name}'>
      <pose>#{_x} #{_y} #{_z} 0 0 #{_yaw}</pose>
      <static>true</static>
      <link name='link'>
        <pose>0 0 0.05 0 0 0</pose>
        <visual name='visual'>
          <geometry>
            <box>
              <size>.1 .1 .1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    </sdf>
  </spawn>
  HEREDOC

  exec = <<-HEREDOC
  <executable name='teambase_description'>
    <command>roslaunch --wait subt_ros teambase_description.launch world_name:=#{$worldName} name:=#{_name}</command>
  </executable>
  <executable name='teambase_ros_ign_bridge'>
    <command>roslaunch --wait subt_ros teambase_topics.launch world_name:=#{$worldName} name:=#{_name}</command>
  </executable>
  HEREDOC

  return spawn, exec
end

def spawnRobot(_robot, _world, _allowLocalModel=false)
  name = _robot.name
  type = _robot.type
  x = _robot.pos.x
  y = _robot.pos.y
  z = _robot.pos.z
  yaw = _robot.rot.yaw
  robotConfigN = _robot.configNumber

  if _robot.isBuiltin
  # we already know it is a builtin robot, so no need to check which particular sensor config
    if type == "X1"
      return spawnX1(name, robotConfigN, x, y, z, yaw)
    elsif type == "X2"
      return spawnX2(name, robotConfigN, x, y, z, yaw)
    elsif type == "X3"
      return spawnX3(name, robotConfigN, x, y, z, yaw)
    elsif type == "X4"
      return spawnX4(name, robotConfigN, x, y, z, yaw)
    elsif type.upcase == "TEAMBASE"
      return spawnTeamBase(name, x, y, z, yaw)
    else
      raise "Unknown builtin robot type: #{_robot.config}"
    end
  else
    spawnerScript = _robot.spawnScriptPath()
    begin
      load spawnerScript
    rescue LoadError
      raise "Unknown robot configuration #{_robot.config}. #{spawnerScript} could not be found."
    else
      modelURI = _robot.modelURI(_allowLocalModel)
      spawnString = spawner(name, modelURI, _world.name,
        _robot.pos.x, _robot.pos.y, _robot.pos.z, 0, 0, _robot.rot.yaw)
      executables = rosExecutables(name, _world.name)
      return spawnString, executables
    end
  end
end

def getMarsupialSnippets(_parentRobot, _childRobot, _worldName)
  childModelName = _childRobot.name
  parentModelName = _parentRobot.name

  detachableJointPlugin = REXML::Document.new <<-HEREDOC
    <plugin filename="libignition-gazebo-detachable-joint-system.so" name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>#{MARSUPIAL_PARENT_LINK_NAMES[_parentRobot.type]}</parent_link>
    <child_model>#{childModelName}</child_model>
    <child_link>base_link</child_link>
    <topic>/model/#{childModelName}/detach</topic>
    <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  HEREDOC

  joints = [detachableJointPlugin]

  platform = ""

  # This will be used to attach the drone platform to the base.
  if _parentRobot.type.include?("X1")
    platformJointPlugin = REXML::Document.new <<-HEREDOC
      <plugin filename="libignition-gazebo-detachable-joint-system.so" name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>#{_parentRobot.name}_platform</child_model>
      <child_link>base_link</child_link>
      <suppress_child_warning>true</suppress_child_warning>
      </plugin>
    HEREDOC

    joints.push(platformJointPlugin)

    # This will be used to spawn the drone platform.
    platform = <<-HEREDOC
      <spawn name="#{_parentRobot.name}_platform">
        <name>#{_parentRobot.name}_platform</name>
        <allow_renaming>false</allow_renaming>
        <pose>#{_parentRobot.pos.x + 0.078} #{_parentRobot.pos.y} #{_parentRobot.pos.z-0.11} 0 0 0</pose>
        <world>#{_worldName}</world>
        <is_performer>false</is_performer>
        <sdf version='1.6'>
          <include>
            <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/DronePlatformX1</uri>
          </include>
        </sdf>
      </spawn>
    HEREDOC
  end

  rosExecutable = <<-HEREDOC
    <executable name="#{childModelName}_marsupial">
      <command>roslaunch --wait subt_ros marsupial_topics.launch name:=#{childModelName}</command>
    </executable>
  HEREDOC

  return joints, platform, rosExecutable
end