def _spawner2(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins='')
  base_spawn = `rospack find bosdyn_spot`.chomp + "/launch/common.rb"
  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{File.dirname(__FILE__)}. #{base_spawn} could not be found."
  else
    additionalSpawnPlugins = <<-HEREDOC
      <!--plugin>custom plugins for this sensor config go here</plugin-->
    HEREDOC
    _pid=[280, 0.3, 7]
    _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, additionalSpawnPlugins + _additionalSpawnPlugins, )
  end
end

def _rosExecutables2(_name, _worldName)
  # description.launch is included from vehicle_topics.launch
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_spot_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=0</command>
  </executable>
  HEREDOC
end
