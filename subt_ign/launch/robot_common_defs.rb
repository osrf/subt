# This is a common file intended to be included in all robot spawner scripts.

$fuelPrefix = "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models"

Vector3d = Struct.new(:x, :y, :z)
AngularVector3d = Struct.new(:roll, :pitch, :yaw)

class Robot
  attr_accessor :name, :config, :type, :configNumber, :pos, :rot, :isBuiltin, :package
  def initialize(name, config)
    @name = name
    @config = config
    match = config.match(/(.*)_SENSOR_CONFIG/)
    if match.nil?
      @type = config
    else
      @type = match[1] unless match.nil?
    end
    @configNumber = config[-1]
    @package = config.downcase

    @pos = Vector3d.new(0, 0, 0)
    @rot = AngularVector3d.new(0, 0, 0)

    @isBuiltin = false
    if @type == "X1" and ["1", "2", "3", "4", "5", "7", "8"].include?(@configNumber)
      @isBuiltin = true
    elsif @type == "X2" and ["1", "2", "3", "4", "5", "6", "8", "9"].include?(@configNumber)
      @isBuiltin = true
    elsif @type == "X3" and ["1", "2", "3", "4"].include?(@configNumber)
      @isBuiltin = true
    elsif @type == "X4" and ["1", "2", "3", "4", "5"].include?(@configNumber)
      @isBuiltin = true
    elsif @type.upcase == "TEAMBASE"
      @isBuiltin = true
    end
  end

  def fuelName()
    # check if robot name follows subt robot naming convention on Fuel.
    # if so, construct a name without underscores, e.g. 'X1 Config 2' or
    # 'X3 UAV Config 4', else use original name
    convention = /^x[[:digit:]]_.*[[:digit:]]/.match(@package)
    if convention
      if @type == "X3" or @type == "X4"
        mUAVStr = "UAV "
      else
        mUAVStr = ""
      end
      robotName = "#{@type} #{mUAVStr}Config #{@configNumber}"
    else
      robotName = @config
    end
    return robotName
  end

  def installDir()
    dir = `rospack find #{@package}`.chomp
    if dir.empty?
      raise "Unknown robot configuration #{@config}. ROS package #{@package} could not be found."
    end
    return dir
  end

  def modelURI(_allowLocalModel = false)
    # We assume that the model and its supporting files can be found in
    # `installDir()`. If `_localModel` is not set, the model is loaded from
    # Fuel.
    if _allowLocalModel
      return installDir()
    else
      return "#{$fuelPrefix}/#{fuelName()}"
    end
  end

  def spawnScriptPath()
    return "#{installDir()}/launch/spawner.rb"
  end

  # This is a snippet common to all robots in the competition. It adds a gas
  # sensor and robot state publishers.
  # @param [Boolean] _enableGroundTruth Whether to allow publishing of ground truth data.
  # @return [String] The snippet to be included in spawner SDF.
  def commonPlugins(_enableGroundTruth)
    return <<-HEREDOC
      <!-- Publish robot state information -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_sensor_pose>true</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>1</static_update_frequency>
      </plugin>

      <!-- Publish ground truth data if allowed -->
      <plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_sensor_pose>false</publish_sensor_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>#{_enableGroundTruth}</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>false</static_publisher>
      </plugin>

      <!-- Gas Sensor plugin -->
      <plugin filename="libGasEmitterDetectorPlugin.so"
        name="subt::GasDetector">
        <topic>/model/#{@name}/gas_detected</topic>
        <update_rate>10</update_rate>
        <type>gas</type>
      </plugin>
    HEREDOC
  end

  # This is a snippet common to all robots carrying breadcrumbs.
  # @param [int] _maxBreadcrumbs The initial number of breadcrumbs the robot will have (can be 0).
  # @param [String] _breadcrumbDropOffset The offset at which a breadcrumb will drop relative to the
  #   current position of the robot's base link; the value is a string with 6 numbers
  #   similar to other pose specifications in SDF
  # @return [String] The snippet to be included in spawner SDF.
  def breadcrumbsPlugins(_maxBreadcrumbs, _breadcrumbDropOffset)
    return <<-HEREDOC
      <!-- Breadcrumbs -->
      <plugin filename="libignition-gazebo-breadcrumbs-system.so"
        name="ignition::gazebo::systems::Breadcrumbs">
        <topic>/model/#{@name}/breadcrumb/deploy</topic>
        <max_deployments>#{_maxBreadcrumbs}</max_deployments>
        <disable_physics_time>3.0</disable_physics_time>
        <topic_statistics>true</topic_statistics>
        <breadcrumb>
          <sdf version="1.6">
            <model name="#{@name}__breadcrumb__">
              <pose>#{_breadcrumbDropOffset}</pose>
              <include>
                <uri>#{$fuelPrefix}/Breadcrumb Node</uri>
              </include>
            </model>
          </sdf>
        </breadcrumb>
      </plugin>
    HEREDOC
  end
end

MARSUPIAL_PARENT_LINK_NAMES = {
   "X1" => "base_link",
   "EXPLORER_X1" => "base_link",
   "EXPLORER_R2" => "Rear_Rocker_Link",
   "CSIRO_DATA61_OZBOT_ATR" => "base_link"
 }

MARSUPIAL_VALID_ROBOT_PAIRS = {
  "X1" => ["X3", "MARBLE_QAV500", "CERBERUS_GAGARIN", "CERBERUS_RMF", "COSTAR_SHAFTER"],
  "EXPLORER_X1" =>["X3", "MARBLE_QAV500", "CERBERUS_GAGARIN", "CERBERUS_RMF", "COSTAR_SHAFTER"],
  "EXPLORER_R2" =>["X3", "X4", "SSCI_X4", "SOPHISTICATED_ENGINEERING_X4", "EXPLORER_DS1", "MARBLE_QAV500", "CERBERUS_M100", "CERBERUS_GAGARIN", "CERBERUS_RMF", "COSTAR_SHAFTER"],
  "CSIRO_DATA61_OZBOT_ATR" =>["X3", "X4", "SSCI_X4", "SOPHISTICATED_ENGINEERING_X4","EXPLORER_DS1", "MARBLE_QAV500", "CERBERUS_M100", "CERBERUS_GAGARIN", "CERBERUS_RMF", "COSTAR_SHAFTER"]
}

MARSUPIAL_PARENT_ROBOT_POSITION_OFFSETS = {
  "X1" => Vector3d.new(0, 0, 0.717),
  "EXPLORER_X1" => Vector3d.new(0, 0, 0.717),
  "EXPLORER_R2" => Vector3d.new(-0.402375, 0, 0.6874),
  "CSIRO_DATA61_OZBOT_ATR" => Vector3d.new(-0.15, 0, 0.528)
}

MARSUPIAL_PARENT_ROBOT_ROTATION_OFFSETS = {
  "X1" => AngularVector3d.new(0, 0, 0),
  "EXPLORER_X1" => AngularVector3d.new(0, 0, 0),
  "EXPLORER_R2" => AngularVector3d.new(0, 0, -3.1416),
  "CSIRO_DATA61_OZBOT_ATR" => AngularVector3d.new(0, 0, -3.1416)
}

def parseRobotArgs(_binding)
  # Check if robotNameX and robotConfigX exists
  robots = Hash.new
  for i in 1..$maxRobotCount do
    if (_binding.local_variables.include?(:"robotName#{i}") &&
        _binding.local_variables.include?(:"robotConfig#{i}"))
      name=_binding.eval("robotName#{i}")
      config=_binding.eval("robotConfig#{i}")
      if name != nil && !name.empty?
        raise "Duplicate robot name #{name}" if robots.has_key? name
        robots[name] = Robot.new(name, config)
      end
    end
  end
  fail ArgumentError, "missing robotNameX or robotConfigX arguments" if robots.empty?

  return robots
end

def parseMarsupials(_binding, _robots)
  # key: parent, value: child
  marsupialParents = Hash.new

  # The logic for validating marsupial robots is:
  # * Check that parent names are unique
  # * Check that child names are unique
  # * Check that each parent and child robot is in the list of robots to be spawned
  # * Check that parent robots are in the MARSUPIAL_VALID_ROBOT_PAIRS list
  # * Check that child robots are in the MARSUPIAL_VALID_ROBOT_PAIRS list
  # These criteria ensure that the robots specified in the "marsupial" variables are valid
  # and that there is a one-to-one relationship between parent and child robots

  # Handle marsupial robots
  for i in 1..$maxRobotCount do
    if _binding.local_variables.include?(:"marsupial#{i}")
      parent, child = _binding.eval("marsupial#{i}").split(':')

      # Check that parent names are unique
      if marsupialParents.has_key?(parent)
        raise "Invalid marsupial configuration: The parent robot [#{parent}] cannot be used more than once"
      end

      # Check that child names are unique
      if marsupialParents.has_value?(child)
        raise "Invalid marsupial configuration: The child robot [#{child}] cannot be used more than once"
      end

      # Check that each parent and child robot is in the list of robots to be spawned
      unless _robots.has_key?(parent)
        raise "Invalid marsupial configuration: The parent robot [#{parent}] is not in the list of robots to be spawned"
      end
      unless _robots.has_key?(child)
        raise "Invalid marsupial configuration: The child robot [#{child}] is not in the list of robots to be spawned"
      end

      parentType = _robots[parent].type
      childType = _robots[child].type

      # Check that parent robots are in the MARSUPIAL_VALID_ROBOT_PAIRS hash
      unless MARSUPIAL_VALID_ROBOT_PAIRS.key?(parentType)
        raise "Invalid marsupial configuration: The parent robot [#{parent}] with type [#{parentType}] is not in the list of robots allowed to be marsupial parents. The list is #{MARSUPIAL_VALID_ROBOT_PAIRS}"
      end
      # Check that child robots are in the MARSUPIAL_VALID_ROBOT_PARIS hash
      unless MARSUPIAL_VALID_ROBOT_PAIRS[parentType].include?(childType)
        raise "Invalid marsupial configuration: The child robot [#{child}] with type [#{childType}] is not in the list of robots allowed to be marsupial children: The list is #{MARSUPIAL_VALID_ROBOT_PAIRS}"
      end

      marsupialParents[parent] = child
    end
  end
  return marsupialParents
end
