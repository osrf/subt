#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_ws/install/setup.sh

# https://wiki.ros.org/rosconsole#Force_line_buffering_for_ROS_logger
# See also https://bitbucket.org/osrf/subt/issues/204
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

rosbag record /rosout_agg -O ~/.ros/ros.bag &

ign launch cloudsim_bridge.ign -v 4 $@
