def _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF)

  # read robot configuration from the config/ dir to access robot geometry information
  require 'yaml'
  confdir = File.join(__dir__, '..', 'config')
  config_yamls = [ File.join(confdir, 'common.yaml'), File.join(confdir, 'sim.yaml'), File.join(confdir, 'ign.yaml') ]
  config = Hash.new
  for config_yaml in config_yamls
    conf = YAML.load_file(config_yaml)
    config.merge!(conf)
  end

  num_wheels = config["num_wheels"]
  wheel_separation = config["rover_bodyWidth"]/2.0 + config["rover_flipperBodyGap"] + config["rover_flipperWidth"]/2.0
  inflation_coef = config["flipper_inflation_ratio"]
  rover_flipperBigRadius = config["rover_flipperBigRadius"]
  rover_flipperSmallRadius = config["rover_flipperSmallRadius"]

  small_wheel_radius = rover_flipperSmallRadius * inflation_coef
  wheel_radius_increment = (rover_flipperBigRadius - rover_flipperSmallRadius)/(num_wheels-1) * inflation_coef

  max_vel = config["rover_maxTrackSpeed"]
  max_accel = config["rover_maxTrackAcceleration"]

  # generate a bunch of DiffDrive plugins (one for each simulated wheel size)
  diff_drive = ""
  for wheel_num in 1..num_wheels
    # we only want odometry from the first diffdrive
    no_odom = ""
    if wheel_num > 1
      no_odom = "<odom_topic>unused_odom</odom_topic>\n"
    end
    diff_drive += "
      <plugin filename=\"libignition-gazebo-diff-drive-system.so\"
              name=\"ignition::gazebo::systems::DiffDrive\">
          <left_joint>front_left_flipper_wheel#{wheel_num}_j</left_joint>
          <left_joint>rear_left_flipper_wheel#{wheel_num}_j</left_joint>
          <right_joint>front_right_flipper_wheel#{wheel_num}_j</right_joint>
          <right_joint>rear_right_flipper_wheel#{wheel_num}_j</right_joint>
          <wheel_separation>#{wheel_separation}</wheel_separation>
          <wheel_radius>#{small_wheel_radius + wheel_radius_increment * (num_wheels - wheel_num)}</wheel_radius>
          <topic>/model/#{_name}/cmd_vel_relay</topic>
          <min_velocity>#{-max_vel}</min_velocity>
          <max_velocity>#{max_vel}</max_velocity>
          <min_acceleration>#{-max_accel}</min_acceleration>
          <max_acceleration>#{max_accel}</max_acceleration>
          #{no_odom}
      </plugin>"
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
      #{diff_drive}
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
        <power_load>9.9</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
      <!-- Wheel slip is in SDF -->
      #{_additionalSDF}
    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def _rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='description'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 description.launch name:=#{_name}</command>
  </executable>
  HEREDOC
end
