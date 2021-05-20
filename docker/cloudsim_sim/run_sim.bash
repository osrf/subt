#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_ws/install/setup.sh

mkdir -p /tmp/ign/logs

# Network traffic monitoring
FILE=/tmp/ign/logs/network_traffic.csv
echo "unix timestamp;iface_name;bytes_out/s;bytes_in/s;bytes_total/s;bytes_in;bytes_out;packets_out/s;packets_in/s;packets_total/s;packets_in;packets_out;errors_out/s;errors_in/s;errors_in;errors_out" > $FILE
bwm-ng -o csv -c 0 -t 1000 -T rate -I eth0 >> $FILE &

# atop process monitoring
atop -R -w /tmp/ign/logs/atop_log &

export ROS_LOG_DIR=/tmp/ign/logs/ros
ign launch -v 4 "$@" 2>&1 | tee /tmp/ign/logs/gzserver.log
