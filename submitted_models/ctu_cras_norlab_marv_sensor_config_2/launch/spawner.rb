require 'yaml'
confdir = File.join(__dir__, '..', 'config')
config = YAML.load_file(File.join(confdir, 'sim.yaml'))

$num_breadcrumbs = config["num_breadcrumbs"]

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF='')
  base_spawn = `rospack find ctu_cras_norlab_marv_sensor_config_1`.chomp + "/launch/common.rb"
  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{config}. #{spawnerScript} could not be found."
  else
    _additionalSDF = _additionalSDF + <<-HEREDOC
      <plugin filename="ignition-gazebo-breadcrumbs-system"
            name="ignition::gazebo::systems::Breadcrumbs">
        <topic>/model/#{_name}/breadcrumb/deploy</topic>
        <max_deployments>#{$num_breadcrumbs}</max_deployments>
        <disable_physics_time>3.0</disable_physics_time>
        <topic_statistics>true</topic_statistics>
        <breadcrumb>
          <sdf version="1.6">
            <model name="#{_name}__breadcrumb__">
              <pose>-0.45 -0.13 0 0 0 #{Math::PI/2}</pose>
              <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Breadcrumb Node</uri>
             </include>
           </model>
         </sdf>
       </breadcrumb>
      </plugin>
    HEREDOC
    _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF)
  end
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} breadcrumbs:=#{$num_breadcrumbs}</command>
  </executable>
  <executable name='description'>
    <command>roslaunch --wait ctu_cras_norlab_marv_sensor_config_1 description.launch name:=#{_name} print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
