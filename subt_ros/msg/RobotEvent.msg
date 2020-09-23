# An event representing an interesting occurrence for an individual robot

time timestamp 		# time since sim bootup
string event_type	# type of event ("detach" for marsupials, "deploy" for breadcrumbs, "dead_battery", or "flip")
string robot_name	# name of the robot that triggered the event
