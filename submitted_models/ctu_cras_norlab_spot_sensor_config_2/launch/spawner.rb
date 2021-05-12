require 'yaml'
confdir = File.join(__dir__, '..', 'config', 'model')
config = YAML.load_file(File.join(confdir, 'common.yaml'))

$num_breadcrumbs = config["num_breadcrumbs"]

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  base_spawn = `rospack find ctu_cras_norlab_spot_sensor_config_1`.chomp + "/launch/common.rb"
  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{File.dirname(__FILE__)}. #{base_spawn} could not be found."
  else
    additionalSpawnPlugins = <<-HEREDOC
      <plugin filename="ignition-gazebo-breadcrumbs-system"
            name="ignition::gazebo::systems::Breadcrumbs">
        <topic>/model/#{_name}/breadcrumb/deploy</topic>
        <max_deployments>#{$num_breadcrumbs}</max_deployments>
        <disable_physics_time>3.0</disable_physics_time>
        <topic_statistics>true</topic_statistics>
        <breadcrumb>
          <sdf version="1.6">
            <model name="#{_name}__breadcrumb__">
              <pose>-0.6 0 0 -0.3 0 #{Math::PI/2}</pose>
              <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Breadcrumb Node</uri>
             </include>
           </model>
         </sdf>
       </breadcrumb>
      </plugin>
    HEREDOC
    _spawner2(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, additionalSpawnPlugins)
  end
end

def rosExecutables(_name, _worldName)
  # description.launch is included from vehicle_topics.launch
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_spot_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=#{$num_breadcrumbs} description_print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
