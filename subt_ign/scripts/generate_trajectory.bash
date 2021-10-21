#!/bin/bash
#
# Usage: ./generate_trajectory.bash <path_to_state_log_file> <video_output_path> <world_name> "<camera_pose>"
#
# Example usage:
#
# ./generate_trajectory.bash ~/Downloads/log/gazebo /tmp/output/ final_event_01 "195 0 317"
#
# Notes:
#   1. You will need the path_tracer executable in your PATH.
#   2. Requires Ignition Fortress
#   3. A set of decent camera poses for some worlds:
#          final_prelim_01 "163 30 310"
#          final_prelim_02 "140 -100 400"
#          final_prelim_03 "90 0 260"
#          final_event_01 "195 0 317"
#          final_event_02 "91 32 300"
#          final_event_03 "247 0 421"
#          final_event_04 "161 140 382"
#          final_event_05 "30 30 100"
#          final_event_06 "79 95 367"
#          final_event_07 "100 -60 410"
#          final_event_08 "25 58 180"

logDir=$1
destDir=$2
worldName=$3
cameraPose=$4
partition=`hostname`"_PATH_TRACER"

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
