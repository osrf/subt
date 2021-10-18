#!/bin/bash
#
# Usage: ./generate_trajectory.bash <path_to_log_files> <output_path> <world_name> <camera_pose>
#
# Example usage:
#
# ./generate_trajectory.bash ~/Downloads/log /tmp/output/ cave_qual
#
# Notes:
#   1. You will need the path_tracer executable in your PATH.
#   2. Requires Ignition Citadel
#   3. If you kill this before docker full starts, then use
#      `docker kill <ID>` to kill the docker container.

# Start clean
# pkill -f "path_tracer"
# pkill -f "ign gazebo"
# docker kill $(docker ps -q)
# pkill -f "ign gazebo"

logDir=$1
destDir=$2
worldName=$3
cameraPose=$4
partition=`hostname`"_PATH_TRACER"

export IGN_TRANSPORT_RCVHWM=0 
export IGN_TRANSPORT_SNDHWM=0 
export IGN_TRANSPORT_TOPIC_STATISTICS=1

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


# Run the simulation world.
docker run -t \
  -l "subt-path" \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e IGN_PARTITION=$partition \
  -e IGN_TRANSPORT_RCVHWM=0 \
  -e IGN_TRANSPORT_SNDHWM=0 \
  -e IGN_TRANSPORT_TOPIC_STATISTICS=1 \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  -v "$destDir:/tmp/ign" \
  --network host \
  --rm \
  --privileged \
  --security-opt seccomp=unconfined \
  --gpus all \
  osrf/subt-virtual-testbed:cloudsim_sim_fortress \
  -v 4 path_tracer.ign worldName:=$worldName &

# Wait for docker to start
sleep 120 

# Get the docker ID
dockerid=`docker ps -f "label=subt-path" --format "{{.ID}}"`

# SIGNINT handler
trap handler INT
function handler() {
  # kill the docker container
  docker kill $dockerid 
  exit 1
}

echo "# Starting trajectory for $logDir" >> record_status.log
# Run the path tracer.
path_tracer $logDir ./path_tracer.yml $partition "$cameraPose" $worldName
echo "# Finished trajectory for $logDir" >> record_status.log

# kill the docker container
docker stop $dockerid 
