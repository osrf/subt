require 'yaml'

def _spawner3(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF='')
  base_spawn = `rospack find ctu_cras_norlab_absolem_sensor_config_3`.chomp + "/launch/common.rb"

  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{_modelURI}. #{base_spawn} could not be found."
  else
    _spawner2(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF)
  end
end

def _rosExecutables3(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} revision:=2021 has_cliff_sensors:=1 has_omnicam:=0 has_omnicam_vras:=1 description_print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
