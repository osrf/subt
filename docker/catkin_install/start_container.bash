#!/usr/bin/env bash

if [ $# -ne 1 ]
then
  echo "Usage: $0 PATH_TO_REPOS"
  exit 1
fi

# load repo into the workspace as a volume
CONTAINER_WS_PATH="/home/dev/subt_ws/src/"
WS_DIR=$1
echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH"
REPO_VOLUME_MOUNT="-v $WS_DIR:$CONTAINER_WS_PATH"

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# need privileged flag or else things like rviz fail:
# https://answers.ros.org/question/301056/ros2-rviz-in-docker-container/
# https://github.com/moby/moby/issues/38442
docker run -it \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env="DISPLAY" \
  --user="dev" \
  --name subt_catkin_container \
  --network host \
  --privileged \
  --rm \
  --runtime=nvidia \
  $REPO_VOLUME_MOUNT \
  subt_catkin_install:latest