#!/usr/bin/env bash

# This is used by the score.test rostest.
ign launch -v 4 competition_no_ros.ign headless:=true robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1 robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG2
