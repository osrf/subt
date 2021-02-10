# Position, velocity, and speed for a robot relative to the artifact origin. Updated when the robot moves > 1 m.

std_msgs/Header header			# sequence number, time since sim bootup, and coordinate frame
geometry_msgs/Pose pose			# position and orientation of the robot
geometry_msgs/Vector3 velocity		# velocity of the robot
float64 speed				# speed of the robot (magnitude of the velocity vector)
