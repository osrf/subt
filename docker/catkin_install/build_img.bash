#!/usr/bin/env bash

docker build -t subt_catkin_install --build-arg user_id=$(id -u) .