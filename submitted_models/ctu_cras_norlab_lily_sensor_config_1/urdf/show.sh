#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo -e "Usage: show [FILENAME]\n FILENAME .. urdf file to be visualized in rviz"
    exit 1
fi

echo "processing $1"
roslaunch urdf_tutorial display.launch model:=$1
