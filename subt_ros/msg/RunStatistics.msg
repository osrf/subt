# Competition statistics for the current run

time timestamp						# time since sim bootup

string world_name			# name of the simulation world
subt_ros/Robot[] robots					# list of robot names, platforms, types
subt_ros/Marsupial[] marsupials				# list of marsupial robot pairs
int32 artifacts_found					# number of artifacts successfully scored (aka current score)
int32 robot_count					# number of robots
int32 unique_robot_count				# number of unique base robot platforms utilized

int32 sim_time_elapsed					# sim time since the run started
int32 real_time_elapsed					# real time since the run started
int32 artifact_report_count				# number of unique artifact reports received
int32 duplicate_report_count				# number of duplicate artifact reports received

# the successfully scored artifact report with the least positional error
string closest_artifact_report_name			# artifact model name (e.g., backpack_4)
string closest_artifact_report_type			# artifact type (e.g., TYPE_BACKPACK)
geometry_msgs/Vector3 closest_artifact_report_true_pos	# true position of the artifact, xyz [m]
geometry_msgs/Vector3 closest_artifact_report_reported_pos # reported position of the artifact [m]
float64 closest_artifact_report_distance		# distance error between true and reported positions [m]

# time of the first and last successfully scored artifact reports (sim time since bootup)
float64 first_artifact_report_time
float64 last_artifact_report_time
float64 mean_time_between_successful_artifact_reports	# avg time between successful artifact reports

# distance traversed by robots
subt_ros/RobotMetric greatest_distance_traveled		# individual robot with the largest distance traversed
subt_ros/RobotMetric greatest_euclidean_distance_from_start	# individual robot that traveled the furthest from staging area
float64 total_distance_traveled				# total distance traveled by all robots
subt_ros/RobotMetric greatest_max_vel			# individual robot with the greatest maximum speed
subt_ros/RobotMetric greatest_avg_vel			# individual robot with the greatest average speed

# elevation traversed by robots, calculated with each increment of 5m
subt_ros/RobotMetric greatest_elevation_gain		# individual robot with the greatest elevation gain
subt_ros/RobotMetric greatest_elevation_loss		# individual robot with the greatest elevation loss
float64 total_elevation_gain				# total elevation gain across all robots
float64 total_elevation_loss				# total elevation loss across all robots
subt_ros/RobotMetric max_elevation_reached		# individual robot that traveled to the highest elevation relative to staging area
subt_ros/RobotMetric min_elevation_reached		# individual robot that traveled to the lowest elevation relative to staging area
