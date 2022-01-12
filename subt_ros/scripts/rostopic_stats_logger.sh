#!/bin/bash

source /opt/ros/noetic/setup.bash

rostopic echo /statistics > $HOME/.ros/rostopic_stats_logger.log
