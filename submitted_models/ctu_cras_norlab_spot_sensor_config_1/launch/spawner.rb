def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  additionalSpawnPlugins = <<-HEREDOC
  <!--plugin>custom plugins for this sensor config go here</plugin-->
  HEREDOC

  begin
    require `rospack find bosdyn_spot`.chomp + "/launch/spawner.rb"
    spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, additionalSpawnPlugins)
  end
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='description'>
    <command>roslaunch --wait ctu_cras_norlab_spot_sensor_config_1 description.launch name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_spot_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=0</command>
  </executable>
  HEREDOC
end
