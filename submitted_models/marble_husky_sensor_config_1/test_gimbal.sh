#!/bin/bash

### Move pan and tilt axes and set back near start position:
rostopic pub -1 /X1/pan_tilt/tilt_rate_cmd_double std_msgs/Float64 -- 3.0
rostopic pub -1 /X1/pan_tilt/tilt_rate_cmd_double std_msgs/Float64 -- -3.0
rostopic pub -1 /X1/pan_tilt/tilt_rate_cmd_double std_msgs/Float64 -- 0.0
rostopic pub -1 /X1/pan_tilt/pan_rate_cmd_double std_msgs/Float64 -- 3.0
rostopic pub -1 /X1/pan_tilt/pan_rate_cmd_double std_msgs/Float64 -- -3.0
rostopic pub -1 /X1/pan_tilt/pan_rate_cmd_double std_msgs/Float64 -- 0.0
