#!/bin/bash
#
# Captures 5 thumbnails (iso, top, side, front, rear) for a model. Stores thumbnails in ~/model_thumbnails/<MODEL NAME>
#
# Requirements: "shutter" installed, "blank.sdf" world
# Note the "blank.sdf" world has custom lighting and white background. Another world could be used, but the world *must* have the ignition UserCommands plugin. See the "blank.sdf" world for example.
# 
# Usage: 	
#		ign gazebo -v 4 blank.sdf 
#		./capture_thumbnails "X1 Config 1" 0.3 
# 		(./capture_thumbnails <MODEL NAME> <ZOOM>)
#
# Recommended zoom values:
#     * 0.50 for X1
#     * 0.35 for X2
#     * 0.27 for X3
#     * 0.32 for X4
# Screenshot location values (window_x, window_y) may need to be adjusted based on your screen size and the model you capture.

if [ -z "$1" ]
then
	printf "No model entered.\nUsage: ./capture_thumbnails <MODEL NAME> <ZOOM>\n"
	exit
else
	model=$1
	dir="$HOME/model_thumbnails/$model/thumbnails"
	size="800,600" # width, height in pixels (do not change), SubT Virtual Portal uses this aspect ratio
	window_x=317
	window_y=282
	sides_window_y=$(($window_y-120)) # adjust Y for side screenshots due to focus on ground plane
	zoom=$2	
	world=blank
	printf "Capturing thumbnails for $model in $dir\n"
	mkdir -p "$dir"
fi

# Spawn the model at origin
printf "Spawn model $model at origin\n"
ign service -s /world/$world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 5000 --req 'sdf: ''"<?xml version=\"1.0\" ?>''<sdf version=\"1.6\">''<include>''<name>model</name>'"<uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/$model</uri>"'</include>''</sdf>' >/dev/null 2>&1
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x:0 y:0 z:0' >/dev/null
#sleep 1

# Iso view screenshot
printf "Iso view.\n"
ign service -s /gui/move_to --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "model"' >/dev/null
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req "x:-$zoom y:$zoom z:-$zoom" >/dev/null
shutter -s="$window_x,$window_y,$size" -e -n -o "$dir"/1.png >/dev/null 2>&1

# Top view screenshot
printf "Top view.\n"
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x:0 y:0 z:-1' >/dev/null
shutter -s="$window_x,$window_y,$size" -e -n -o "$dir"/2.png >/dev/null 2>&1

# Side view screenshot
printf "Side view.\n"
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x:0 y:0.9 z:0' >/dev/null
shutter -s="$window_x,$sides_window_y,$size" -e -n -o "$dir"/3.png >/dev/null 2>&1

# Front view screenshot
printf "Front view.\n"
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x:-1 y:0 z:0' >/dev/null
shutter -s="$window_x,$sides_window_y,$size" -e -n -o "$dir"/4.png >/dev/null 2>&1

# Rear view screenshot
printf "Rear view.\n"
ign service -s /gui/view_angle --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x:1 y:0 z:0' >/dev/null
shutter -s="$window_x,$sides_window_y,$size" -e -n -o "$dir"/5.png >/dev/null 2>&1

# Remove the model
printf "Remove model.\n"
ign service -s /world/$world/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 2000 --req 'name: "model" type: MODEL' >/dev/null

printf "Captured thumbnails for $model in $dir\n"
