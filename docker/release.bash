#!/bin/bash

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

# release.sh: A shell script to build, tag and push SubT Docker images.
#
# E.g.: ./release.sh --build
# E.g.: ./release.sh --tag
# E.g.: ./release.sh --push
# E.g.: ./release.sh --all

set -e

# Define usage function.
usage()
{
  echo "Usage: $0 [--build | --tag | --push | --all]"
  echo -e "Important: If you're a competitor, this script is not relevant for you\n"
  echo "Available commands:"
  echo "  --build    Build all SubT Docker images"
  echo "  --tag      Tag all SubT Docker images"
  echo "  --push     Push all SubT Docker images to DockerHub"
  echo -e "  --all      Build, tag and push all SubT Docker images\n"
  echo "Typically, you should:"
  echo "  1. Log in to a Docker registry. E.g.: docker login"
  echo "  2. Build your new Docker images"
  echo "  3. Test the images"
  echo "  4. Tag the images"
  echo "  5. Push the images"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -ne 1 ]] && usage

COMMAND=$1

# Sanity check: Make sure that the parameter is supported.
if [[ "$COMMAND" != "--build" ]] && [[ "$COMMAND" != "--tag" ]] &&
   [[ "$COMMAND" != "--push" ]]  && [[ "$COMMAND" != "--all" ]]; then
  usage
fi

# Build the images.
if [[ "$COMMAND" == "--build" ]] || [[ "$COMMAND" == "--all" ]]; then
  ./build.bash cloudsim_sim --no-cache
  ./build.bash cloudsim_bridge --no-cache
  ./build.bash subt_sim_entry --no-cache
fi

# Tag the images.
if [[ "$COMMAND" == "--tag" ]] || [[ "$COMMAND" == "--all" ]]; then
  docker tag cloudsim_sim:latest nkoenig/subt-virtual-testbed:cloudsim_sim_latest
  docker tag cloudsim_bridge:latest nkoenig/subt-virtual-testbed:cloudsim_bridge_latest
  docker tag subt_sim_entry:latest nkoenig/subt-virtual-testbed:latest
fi

# Push the images.
if [[ "$COMMAND" == "--push" ]] || [[ "$COMMAND" == "--all" ]]; then
  docker push nkoenig/subt-virtual-testbed:cloudsim_sim_latest
  docker push nkoenig/subt-virtual-testbed:cloudsim_bridge_latest
  docker push nkoenig/subt-virtual-testbed:latest
fi
