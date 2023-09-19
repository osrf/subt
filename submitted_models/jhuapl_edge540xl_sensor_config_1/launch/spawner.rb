def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw)
  
  _plantURDF = File.join(File.expand_path("../urdf", File.dirname(__FILE__)), "Edge540XLPlant.urdf")
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
        <topic>/model/#{_name}/input</topic>
        <link_name>fuselage</link_name>
        <min_x>-1000</min_x>
        <max_x>1000</max_x>
        <min_y>-1000</min_y>
        <max_y>1000</max_y>
        <min_z>-1000</min_z>
        <max_z>1000</max_z>
      </plugin>

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
        <voltage>12.6</voltage>
        <open_circuit_voltage_constant_coef>12.6</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
        <initial_charge>2.2</initial_charge>
        <capacity>2.2</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>1.9499</smooth_current_tau>
        <power_load>2</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>

      <!-- Joint state publisher -->
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
      </plugin>

      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{_name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>

      <sensor name="air_pressure" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <air_pressure>
            <reference_altitude>0</reference_altitude>
            <noise type="gaussian">
                <mean>0.00000008</mean>
            </noise>
        </air_pressure>
      </sensor>

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
