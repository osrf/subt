# SubT Repo - Catkin Installation

The files in this directory can be used to follow the [subt catkin install](https://github.com/osrf/subt/wiki/Catkin%20System%20Setup) instructions through a Docker image.

This may be useful for doing things like [breadcrumbs visualization](https://github.com/osrf/subt/wiki/Breadcrumbs-and-communication-visualization-tutorial).

## Installation

```
$ ./build_img.bash
```

## Usage

First, clone the [subt](https://github.com/osrf/subt) and [ros_ign](https://github.com/ignitionrobotics/ros_ign) (optional - needed for breadcrumbs visualization) repositories in the same directory.
This directory with these repositories will be loaded into the Docker container as a volume.
The container's location for this volume is `~/subt_ws/src`, with `~/subt_ws` being the root of the workspace in the container.

To start a container:

```
# PATH/TO/REPOS should be a directory that contains the subt and ros_ign repos
$ ./start_container.bash PATH/TO/REPOS
```

To start another bash session in an existing container:

```
./join.bash
```

### Example Usage - Building a Workspace

Once you are in the container, you may want to run something like the following commands to build the workspace:

```
$ cd ~/subt_ws
$ . /opt/ros/melodic/setup.bash
$ colcon build    # OR run 'catkin_make install'
```

### Example Usage - Breadcrumbs Visualization

Go trough the following steps in order to perform breadcrumbs visualization:

1. Clone the [subt](https://github.com/osrf/subt) and [ros_ign](https://github.com/ignitionrobotics/ros_ign) repositories into a directory, using the right branches.

```
$ cd ~
$ mkdir repos
$ cd repos/
$ git clone --branch citadel https://github.com/osrf/subt.git
$ git clone --branch melodic https://github.com/ignitionrobotics/ros_ign.git
```

2. Start a container that loads these repositories as a volume

```
$ ./start_container.bash ~/repos
```

3. In the container, remove ignition blueprint and install ignition citadel

```
$ sudo apt -y remove ignition-blueprint && sudo apt update && sudo apt -y install ignition-citadel
```

4. Build the workspace. Be sure to source ROS first

```
$ . /opt/ros/melodic/setup.bash
$ cd ~/subt_ws/
$ colcon build
```

5. Go to the [Deploy a breadcrumb and visualize communications](https://github.com/osrf/subt/wiki/Breadcrumbs-and-communication-visualization-tutorial#deploy-a-breadcrumb-and-visualize-communications) section of the breadcrumbs visualization tutorial and complete the remaining steps in that tutorial, performing all of the commands in the docker container (remember that you can use `join.bash` in order to access another shell in the Docker container if needed).

_Calling the breadcrumbs visualization service in the tutorial can take a while, so so don't worry if the service call does not return right away._

## Other Notes

If you need to install new packages in the container (for example: breadcrumbs visualization requires you to install ignition citadel), be sure to run the following command first:

```
$ sudo apt update
```