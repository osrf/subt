#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_ws/install/setup.sh

mkdir -p /home/developer/.ros/config

# Network traffic monitoring
FILE=/home/developer/.ros/network_traffic.csv
echo "unix timestamp;iface_name;bytes_out/s;bytes_in/s;bytes_total/s;bytes_in;bytes_out;packets_out/s;packets_in/s;packets_total/s;packets_in;packets_out;errors_out/s;errors_in/s;errors_in;errors_out" > $FILE
bwm-ng -o csv -c 0 -t 1000 -T rate -I eth0 >> $FILE &

# Set rosccp verbosity to DEBUG
echo "log4j.logger.ros.roscpp.cached_parameters=INFO" >> /home/developer/.ros/config/rosconsole.config
echo "log4j.logger.ros.roscpp=DEBUG" >> /home/developer/.ros/config/rosconsole.config

# https://wiki.ros.org/rosconsole#Force_line_buffering_for_ROS_logger
# See also https://bitbucket.org/osrf/subt/issues/204
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

ign launch cloudsim_bridge.ign -v 4 $@
