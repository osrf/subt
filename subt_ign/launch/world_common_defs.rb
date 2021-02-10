# This is a common file intended to be included in all competition world launchers.

# Library of functions for robots
require File.dirname(__FILE__) + "/robot_common_defs.rb"

$validCircuits = ["cave", "tunnel", "urban"]

$spawnRowSize = 4
$spawnColSize = 5
$spawnGridSize = 2.0
$maxRobotCount = $spawnRowSize * $spawnColSize

class World
  attr_accessor :name, :path, :number, :levels, :circuit, :guiCameraPose, :spawnBasePos, :spawnBaseRot, :commsFadingExponent, :commsScalingFactor, :commsRangePerHop
  def initialize(name, circuit = "")
    @name = name
    @number = name.split('_').last

    @path = "#{name}.sdf"
    if name.include?('tunnel_circuit_') && !name.include?('practice')
      @path = "tunnel_circuit/#{@number}/#{name}.sdf"
    elsif name.include?('urban_circuit_') && !name.include?('practice')
      @path = "urban_circuit/#{@number}/#{name}.sdf"
    elsif name.include?('cave_circuit_') && !name.include?('practice')
      @path = "cave_circuit/#{@number}/#{name}.sdf"
    end

    # disable levels for simple cave worlds
    @levels = true
    if name.include?('simple_cave_') || name.include?('cave_qual') || name.include?('niosh_')
      @levels = false
    end

    if circuit.empty?
      if name.include?('tunnel') || name.include?('niosh')
        circuit = "tunnel"
      elsif name.include?('urban')
        circuit = "urban"
      else
        circuit = "cave"
      end
    end
    @circuit = circuit
    
    if @circuit == "urban"
      @guiCameraPose = "-2.0 27 15 0 0.868 0"
    else
      @guiCameraPose = "-6.3 -4.2 3.6 0 0.268 0.304"
    end

    @spawnBasePos = Vector3d.new(0, 0, 0)
    @spawnBaseRot = AngularVector3d.new(0, 0, 0)
    if @circuit == "urban"
      @spawnBasePos.x = 6
      @spawnBasePos.y = 27
      @spawnBasePos.z = 7.5
    end

    @commsFadingExponent = 2.5
    @commsScalingFactor = 1.0
    @commsRangePerHop = 2.0
    if @circuit == "cave"
      @commsFadingExponent = 1.5
      @commsScalingFactor = 0.55
      @commsRangePerHop = 2.0
    end
  end

  def assignSpawnPositions(_robots, _marsupialParents)
    spawnColOffset = $spawnColSize * $spawnGridSize / 2
    spawnRowOffset = $spawnRowSize * $spawnGridSize / 2
  
    # Reverse hash of marsupials, i.e, key: child, value: parent
    marsupialChildren = _marsupialParents.invert
  
    # Spawn nonmarsupial and parent marsupial robots first. Then spawn child
    # marsupials since they need the positions of their parents
    spawnList = _robots.select { |name| not marsupialChildren.has_key?(name) }.values +
                _robots.select { |name| marsupialChildren.has_key?(name) }.values
  
    robotSpawned = 0
    spawnList.each do |robot|
      name = robot.name
      config = robot.config
      robotType = robot.type
      robotConfigN = robot.configNumber
      posX = -(robotSpawned / $spawnColSize * $spawnGridSize - spawnRowOffset)
      posY = -(robotSpawned % $spawnColSize * $spawnGridSize - spawnColOffset)
  
      # Spawn above parent position if this robot is marsupial child
      if marsupialChildren.has_key?(name)
        parentRobot = _robots[marsupialChildren[name]]
        offsetX = MARSUPIAL_PARENT_ROBOT_POSITION_OFFSETS[parentRobot.type].x
        offsetY = MARSUPIAL_PARENT_ROBOT_POSITION_OFFSETS[parentRobot.type].y
        robot.pos.x = parentRobot.pos.x + Math.cos(-@spawnBaseRot.yaw) * offsetX + Math.sin(-@spawnBaseRot.yaw) * offsetY
        robot.pos.y = parentRobot.pos.y - Math.sin(-@spawnBaseRot.yaw) * offsetX + Math.cos(-@spawnBaseRot.yaw) * offsetY
        robot.pos.z = parentRobot.pos.z + MARSUPIAL_PARENT_ROBOT_POSITION_OFFSETS[parentRobot.type].z
        robot.rot.roll = parentRobot.rot.roll + MARSUPIAL_PARENT_ROBOT_ROTATION_OFFSETS[parentRobot.type].roll
        robot.rot.pitch = parentRobot.rot.pitch + MARSUPIAL_PARENT_ROBOT_ROTATION_OFFSETS[parentRobot.type].pitch
        robot.rot.yaw = parentRobot.rot.yaw + MARSUPIAL_PARENT_ROBOT_ROTATION_OFFSETS[parentRobot.type].yaw
      else
        x = Math.cos(-@spawnBaseRot.yaw) * posX + Math.sin(-@spawnBaseRot.yaw) * posY
        y = -(Math.sin(-@spawnBaseRot.yaw)) * posX + Math.cos(-@spawnBaseRot.yaw) * posY
        robot.pos.x = x + @spawnBasePos.x
        robot.pos.y = y + @spawnBasePos.y
        robot.pos.z = @spawnBasePos.z
        robot.rot.yaw = @spawnBaseRot.yaw
      end
      robotSpawned += 1
    end
  end
  
  def commonLaunchEnvironment()
    return <<-HEREDOC
      <env>
        <name>IGN_GAZEBO_SYSTEM_PLUGIN_PATH</name>
        <value>$LD_LIBRARY_PATH</value>
      </env>
  
      <env>
        <name>IGN_TRANSPORT_TOPIC_STATISTICS</name>
        <value>1</value>
      </env>
    HEREDOC
  end
  
  def commonServerPlugins()
    return <<-HEREDOC
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-physics-system.so"
              name="ignition::gazebo::systems::Physics">
      </plugin>
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-sensors-system.so"
              name="ignition::gazebo::systems::Sensors">
              <render_engine>ogre2</render_engine>
      </plugin>
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-user-commands-system.so"
              name="ignition::gazebo::systems::UserCommands">
      </plugin>
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-scene-broadcaster-system.so"
              name="ignition::gazebo::systems::SceneBroadcaster">
      </plugin>
  
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-imu-system.so"
              name="ignition::gazebo::systems::Imu">
      </plugin>
  
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-magnetometer-system.so"
              name="ignition::gazebo::systems::Magnetometer">
      </plugin>
  
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libignition-gazebo-air-pressure-system.so"
              name="ignition::gazebo::systems::AirPressure">
      </plugin>
    HEREDOC
  end
  
  def guiPlugin()
    return <<-HEREDOC
      <plugin name="ignition::launch::GazeboGui"
            filename="libignition-launch-gazebogui.so">
        <world_name>#{@name}</world_name>
        <window_title>SubT Simulator</window_title>
        <window_icon>#{ENV['SUBT_IMAGES_PATH']}/SubT_logo.svg</window_icon>
        <plugin filename="GzScene3D" name="3D View">
          <ignition-gui>
            <title>3D View</title>
            <property type="bool" key="showTitleBar">false</property>
            <property type="string" key="state">docked</property>
          </ignition-gui>
  
          <engine>ogre2</engine>
          <scene>scene</scene>
          <ambient_light>0.2 0.2 0.2</ambient_light>
          <background_color>0.8 0.8 0.8</background_color>
          <camera_pose>#{@guiCameraPose}</camera_pose>
        </plugin>
        <plugin filename="WorldControl" name="World control">
          <ignition-gui>
            <title>World control</title>
            <property type="bool" key="showTitleBar">false</property>
            <property type="bool" key="resizable">false</property>
            <property type="double" key="height">72</property>
            <property type="double" key="width">121</property>
            <property type="double" key="z">1</property>
  
            <property type="string" key="state">floating</property>
            <anchors target="3D View">
              <line own="left" target="left"/>
              <line own="bottom" target="bottom"/>
            </anchors>
          </ignition-gui>
  
          <play_pause>true</play_pause>
          <step>true</step>
          <start_paused>true</start_paused>
          <service>/world/#{@name}/control</service>
          <stats_topic>/world/#{@name}/stats</stats_topic>
  
        </plugin>
  
        <plugin filename="WorldStats" name="World stats">
          <ignition-gui>
            <title>World stats</title>
            <property type="bool" key="showTitleBar">false</property>
            <property type="bool" key="resizable">false</property>
            <property type="double" key="height">110</property>
            <property type="double" key="width">290</property>
            <property type="double" key="z">1</property>
  
            <property type="string" key="state">floating</property>
            <anchors target="3D View">
              <line own="right" target="right"/>
              <line own="bottom" target="bottom"/>
            </anchors>
          </ignition-gui>
  
          <sim_time>true</sim_time>
          <real_time>true</real_time>
          <real_time_factor>true</real_time_factor>
          <iterations>true</iterations>
          <topic>/world/#{@name}/stats</topic>
        </plugin>
        <!-- Entity tree -->
        <!-- <plugin filename="EntityTree" name="Entity tree">
          <ignition-gui>
            <title>Entity tree</title>
          </ignition-gui>
        </plugin> -->
  
        <!-- Transform Control -->
        <!-- <plugin filename="TransformControl" name="Transform Control">
          <service>/world/#{@name}/gui/transform_mode</service>
        </plugin> -->
      </plugin>
    HEREDOC
  end
  
  def subtServerPlugins(_durationSec, _logPath, _ros)
    return <<-HEREDOC
      <!-- The SubT challenge logic plugin -->
      <plugin entity_name="#{@name}"
              entity_type="world"
              filename="libGameLogicPlugin.so"
              name="subt::GameLogicPlugin">
        <!-- The collection of artifacts to locate -->
        <world_name>#{@name}</world_name>
        <ros>#{_ros}</ros>
  
        <duration_seconds>#{_durationSec}</duration_seconds>
  
        <logging>
          <!-- Use the <path> element to control where to record the log file.
               The HOME path is used by default -->
          <path>#{_logPath}</path>
          <filename_prefix>subt_#{@circuit}</filename_prefix>
          <elevation_step_size>5</elevation_step_size>
        </logging>
      </plugin>
    HEREDOC
  end
  
  def subtLaunchPlugins()
    return <<-HEREDOC
      <!-- The SubT challenge comms broker plugin -->
      <plugin entity_name="#{@name}"
              entity_type="world"
              name="subt::CommsBrokerPlugin"
              filename="libCommsBrokerPlugin.so">
        <generate_table>true</generate_table>
        <world_name>#{@name}</world_name>
        <comms_model>
          <comms_model_type>visibility_range</comms_model_type>
  
          <range_config>
            <max_range>500.0</max_range>
            <fading_exponent>#{@commsFadingExponent}</fading_exponent>
            <L0>40</L0>
            <sigma>10.0</sigma>
            <scaling_factor>#{@commsScalingFactor}</scaling_factor>
            <range_per_hop>#{@commsRangePerHop}</range_per_hop>
          </range_config>
  
          <visibility_config>
            <visibility_cost_to_fading_exponent>0.2</visibility_cost_to_fading_exponent>
            <comms_cost_max>10</comms_cost_max>
          </visibility_config>
  
          <radio_config>
            <capacity>1000000</capacity>
            <tx_power>20</tx_power>
            <noise_floor>-90</noise_floor>
            <modulation>QPSK</modulation>
          </radio_config>
        </comms_model>
      </plugin>
  
      <!-- The SubT challenge base station plugin -->
      <plugin entity_name="base_station"
              entity_type="model"
              name="subt::BaseStationPlugin"
              filename="libBaseStationPlugin.so">
      </plugin>
    HEREDOC
  end

end
