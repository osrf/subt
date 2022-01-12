#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash
. ~/subt_ws/install/setup.sh

ign launch -v 4 $@
