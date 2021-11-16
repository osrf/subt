def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF='')
  base_spawn = `rospack find ctu_cras_norlab_marv_sensor_config_1`.chomp + "/launch/common.rb"
  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{config}. #{spawnerScript} could not be found."
  else
    _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, '')
  end
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} has_thermal_camera:=1</command>
  </executable>
  <executable name='description'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 description.launch name:=#{_name} print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
