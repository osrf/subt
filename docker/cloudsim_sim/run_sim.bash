#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_ws/install/setup.sh

# Network traffic monitoring
bwm-ng -o csv -u bytes -T rate -C ',' -F /tmp/ign/network_traffic.csv &

ign launch -v 4 $@
