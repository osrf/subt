#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_ws/install/setup.sh

rosbag record /rosout_agg -O ~/.ros/ros.bag &

ign launch cloudsim_bridge.ign -v 4 $@
