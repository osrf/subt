#!/usr/bin/env bash

#
# Copyright (C) 2019 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# This is a wrapper for docker compose creates the necessary xauth file before calling
# docker-compose up.
# Requires:
#   docker
#   docker-compose
#   nvidia-docker
#   an X server

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")

    touch $XAUTH

    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]
then
  echo "[$XAUTH] was not properly created. Exiting..."
  exit 1
fi


if [ $# -gt 0 ]
then
  if [ "${1,,}" == "tunnel" ]
  then
    if [ "${2,,}" == "headless" ]
    then
       echo -e 'circuit=tunnel\nworldName=tunnel_circuit_practice_01\nheadless=true' > .env
    else
       echo -e 'circuit=tunnel\nworldName=tunnel_circuit_practice_01\nheadless=false' > .env
    fi
    docker-compose up
  elif [ "${1,,}" == "urban" ]
  then
    if [ "${2,,}" == "headless" ]
    then
       echo -e 'circuit=urban\nworldName=urban_circuit_practice_01\nheadless=true' > .env
    else
       echo -e 'circuit=urban\nworldName=urban_circuit_practice_01\nheadless=false' > .env
    fi
    docker-compose up
  else
     echo "Invalid [circuit] parameter value. Use tunnel or urban"
  fi
else
  echo "[circuit] parameter value required (e.g. ./run_docker_compose.sh tunnel)"
fi