def _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF, _max_velocity=0.4, _max_acceleration=3)
  # read robot configuration from the config/ dir to access robot geometry information
  require 'yaml'
  confdir = File.join(__dir__, '..', 'config')
  config_yamls = [ File.join(confdir, 'common.yaml'), File.join(confdir, 'sdf.yaml') ]
  config = Hash.new
  for config_yaml in config_yamls
    conf = YAML.load_file(config_yaml)
    config.merge!(conf)
  end

  useWheels = config["wheels_instead_of_tracks"]

  numTrackWheels = 8
  numFlipperWheels = 5

  tracks = ["left", "right"]
  flippers = ["front_left", "front_right", "rear_left", "rear_right"]

  flippersOfTrack = Hash.new
  flippersOfTrack["left"] = ["front_left", "rear_left"]
  flippersOfTrack["right"] = ["front_right", "rear_right"]

  # configuration of diffdrive
  wheelSeparation = 0.397
  wheelRadiuses = [0.089, 0.074, 0.059, 0.044, 0.029]

  # configuration of wheel slip
  slipCompliance = 0.0485
  wheelNormalForce = 10.545

  diffdriveJoints = []
  for i in 0...numFlipperWheels
    diffdriveJoints[i] = ""
  end
  wheelSlip = ""
  trackControllers = ""
  trackLinks = ""

  for track in tracks
    if useWheels
        for wheel in 1..numTrackWheels
          diffdriveJoints[0] += "<#{track}_joint>#{track}_track_wheel#{wheel}_j</#{track}_joint>\n"
          wheelSlip += <<-HEREDOC
            <wheel link_name="#{track}_track_wheel#{wheel}">
              <slip_compliance_lateral>#{slipCompliance}</slip_compliance_lateral>
              <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
              <wheel_normal_force>#{wheelNormalForce}</wheel_normal_force>
              <wheel_radius>#{wheelRadiuses[0]}</wheel_radius>
            </wheel>
          HEREDOC
        end
    else
        trackControllers += <<-HEREDOC
          <plugin name='ignition::gazebo::systems::TrackController' filename='libignition-gazebo-track-controller-system.so'>
              <link>#{track}_track</link>
              <min_velocity>-#{_max_velocity}</min_velocity>
              <max_velocity>#{_max_velocity}</max_velocity>
              <min_acceleration>-#{_max_acceleration}</min_acceleration>
              <max_acceleration>#{_max_acceleration}</max_acceleration>
          </plugin>
        HEREDOC
        trackLinks += "<#{track}_track><link>#{track}_track</link></#{track}_track>"
    end
    for flipper in flippersOfTrack[track]
      if useWheels
          for wheel in 1..numFlipperWheels
            diffdriveJoints[wheel-1] += "<#{track}_joint>#{flipper}_flipper_wheel#{wheel}_j</#{track}_joint>\n"
            wheelSlip += <<-HEREDOC
              <wheel link_name="#{flipper}_flipper_wheel#{wheel}">
                <slip_compliance_lateral>#{slipCompliance}</slip_compliance_lateral>
                <slip_compliance_longitudinal>0</slip_compliance_longitudinal>
                <wheel_normal_force>#{wheelNormalForce}</wheel_normal_force>
                <wheel_radius>#{wheelRadiuses[wheel-1]}</wheel_radius>
              </wheel>
            HEREDOC
          end
      else
        trackControllers += <<-HEREDOC
          <plugin name='ignition::gazebo::systems::TrackController' filename='libignition-gazebo-track-controller-system.so'>
              <link>#{flipper}_flipper</link>
              <min_velocity>-#{_max_velocity}</min_velocity>
              <max_velocity>#{_max_velocity}</max_velocity>
              <min_acceleration>-#{_max_acceleration}</min_acceleration>
              <max_acceleration>#{_max_acceleration}</max_acceleration>
          </plugin>
        HEREDOC
        trackLinks += "<#{track}_track><link>#{flipper}_flipper</link></#{track}_track>"
      end
    end
  end

  diffDrive = ""
  trackedVehicle = ""
  wheelSlipPlugin = ""
  if useWheels
      for wheel in 0...numFlipperWheels
        # we only want odometry from the first diffdrive
        no_odom = ""
        if wheel > 0
          no_odom = "<odom_topic>unused_odom</odom_topic>\n"
        end
        diffDrive += <<-HEREDOC
          <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
            #{diffdriveJoints[wheel]}
            <wheel_separation>#{wheelSeparation}</wheel_separation>
            <wheel_radius>#{wheelRadiuses[wheel]}</wheel_radius>
            <topic>/model/#{_name}/cmd_vel_relay</topic>
            <min_velocity>-#{_max_velocity}</min_velocity>
            <max_velocity>#{_max_velocity}</max_velocity>
            <min_acceleration>-#{_max_acceleration}</min_acceleration>
            <max_acceleration>#{_max_acceleration}</max_acceleration>
            #{no_odom}
          </plugin>
        HEREDOC
      end
      wheelSlipPlugin = <<-HEREDOC
        <plugin filename="ignition-gazebo-wheel-slip-system" name="ignition::gazebo::systems::WheelSlip">
          #{wheelSlip}
        </plugin>
      HEREDOC
  else
    trackedVehicle += <<-HEREDOC
        <plugin name="ignition::gazebo::systems::TrackedVehicle" filename="ignition-gazebo-tracked-vehicle-system">
              #{trackLinks}
              <tracks_separation>#{wheelSeparation}</tracks_separation>
              <tracks_height>0.18094</tracks_height>
              <steering_efficiency>0.5</steering_efficiency>
              <topic>/model/#{_name}/cmd_vel_relay</topic>
              <linear_velocity>
                  <min_velocity>-#{_max_velocity}</min_velocity>
                  <max_velocity>#{_max_velocity}</max_velocity>
                  <min_acceleration>-#{_max_acceleration}</min_acceleration>
                  <max_acceleration>#{_max_acceleration}</max_acceleration>
              </linear_velocity>
          </plugin>
    HEREDOC
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
      <!-- Diff drive -->
      #{diffDrive}
      #{trackedVehicle}
      #{trackControllers}
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
        <power_load>4.95</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
      <!-- Wheel slip -->
      #{wheelSlipPlugin}
      #{_additionalSDF}
    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def _rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
