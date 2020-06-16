#!/bin/bash

source /opt/ros/melodic/setup.bash

rostopic echo /statistics > $HOME/.ros/rostopic_stats_logger.log
