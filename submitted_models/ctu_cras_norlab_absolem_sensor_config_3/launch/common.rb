require 'yaml'

def _spawner2(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF='')
  base_spawn = `rospack find ctu_cras_norlab_absolem_sensor_config_1`.chomp + "/launch/common.rb"

  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{_modelURI}. #{base_spawn} could not be found."
  else
    max_velocity = 0.6
    _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF, max_velocity)
  end
end

def _rosExecutables2(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} revision:=2021 has_cliff_sensors:=1 description_print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
