# An event representing robot entry/exit from an interesting region.

time timestamp		# time since sim bootup
string event_type	# type of event (artifact_proximity, vertical, decision, elevation, rock_fall)
string robot_name	# name of the robot that triggered the event
string detector		# model name defining the detector region (e.g., backpack_1 for an artifact_proximity event, tile_2 for a vertical/decision/elevation event, performer_detector_rockfall_1 for a rock_fall event) 
string state		# enter or exit, represents whether the robot is entering or leaving the region
