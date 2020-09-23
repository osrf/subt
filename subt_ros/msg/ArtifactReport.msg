# An artifact report received from the competing team and associated scoring metrics. Includes only properly-formatted and non-duplicate reports.

time timestamp 					# time since sim bootup
string reported_artifact_type			# artifact type from the report
geometry_msgs/Point reported_artifact_position	# artifact position from the report
string closest_artifact_name			# model name of the nearest artifact, e.g., backpack_4
float64 distance				# distance to nearest artifact [m]
int32 points_scored				# points scored in this attempt
int32 total_score				# resulting total score after scoring this attempt
