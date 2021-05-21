require 'yaml'
confdir = File.join(__dir__, '..', 'config')
config = YAML.load_file(File.join(confdir, 'common.yaml'))

$num_breadcrumbs = config["num_breadcrumbs"]

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  base_spawn = `rospack find ctu_cras_norlab_absolem_sensor_config_6`.chomp + "/launch/common.rb"

  begin
    load base_spawn
  rescue LoadError
    raise "Unknown robot configuration #{_modelURI}. #{base_spawn} could not be found."
  else
    _additionalSDF = <<-HEREDOC
      <plugin filename="ignition-gazebo-breadcrumbs-system" name="ignition::gazebo::systems::Breadcrumbs">
        <topic>/model/#{_name}/breadcrumb/deploy</topic>
        <max_deployments>#{$num_breadcrumbs}</max_deployments>
        <disable_physics_time>3.0</disable_physics_time>
        <topic_statistics>true</topic_statistics>
        <breadcrumb>
          <sdf version="1.6">
            <model name="#{_name}__breadcrumb__">
              <pose>-0.45 0.08 0.1 0 0 #{Math::PI/2}</pose>
              <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Breadcrumb Node</uri>
             </include>
           </model>
         </sdf>
       </breadcrumb>
      </plugin>
    HEREDOC
    _spawner3(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF)
  end
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='topics'>
    <command>roslaunch --wait ctu_cras_norlab_absolem_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name} revision:=2021 has_cliff_sensors:=1 has_omnicam:=0 has_omnicam_vras:=1 breadcrumbs:=1 description_print_command:="#{File.dirname(__FILE__) + '/../scripts/print_robot_urdf'}"</command>
  </executable>
  HEREDOC
end
