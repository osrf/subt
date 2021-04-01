def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _plantURDF='urdf/Edge540XLPlant.urdf')
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

      <!-- Simulate plane dynamics -->
      <plugin filename="libjhuapl_uav_dynamics.so"
        name="jhuapl::Edge540xlDynamics">
        <plant_urdf_file>#{_plantURDF}</plant_urdf_file>
        <model_name>#{_name}</model_name>
        <link_name>fuselage</link_name>
        <min_x>-1000</min_x>
        <max_x>1000</max_x>
        <min_y>-1000</min_y>
        <max_y>1000</max_y>
        <min_z>-1000</min_z>
        <max_z>1000</max_z>
      </plugin>

      <!-- Battery plugin -->
      <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <battery_name>linear_battery</battery_name>
        <voltage>12.4</voltage>
        <open_circuit_voltage_constant_coef>12.4</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>12.3</initial_charge>
        <capacity>12.3</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>5.214</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>

    </include>
    </sdf>
  </spawn>
  HEREDOC
end

def rosExecutables(_name, _worldName)
  <<-HEREDOC
  <executable name='robot_description'>
    <command>roslaunch --wait jhuapl_edge540xl_sensor_config_1 description.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  <executable name='topics'>
    <command>roslaunch --wait jhuapl_edge540xl_sensor_config_1 vehicle_topics.launch world_name:=#{_worldName} name:=#{_name}</command>
  </executable>
  HEREDOC
end
