# DARPA SubT Virtual Competition Software

This repostory contains software for the virtual track of the [DARPA SubT Challenge](https://subtchallenge.com/). Within this repository you will find [Gazebo](http://gazebosim.org) simulation assets, [ROS](http://ros.org) interfaces, support scripts and plugins, and documentation needed to compete in the SubT Virtual Challenge.

![tunnel_action-small.jpg](https://bitbucket.org/repo/8ze6Mjd/images/2265205947-tunnel_action-small.jpg)

## Quick Start

** These instructions are a work in progress **

1. You'll need a computer running [Ubuntu Bionic (18.04)](http://releases.ubuntu.com/18.04/), and a Logitech or Playstation USB joystick.

2. Install a few dependencies which includes Gazebo, ROS, and supporting packages.

    ```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install build-essential cmake git libbluetooth-dev libcwiid-dev libgoogle-glog-dev libspnav-dev libusb-dev lsb-release mercurial pkg-config python gazebo9 libgazebo9-dev ros-melodic-desktop-full ros-melodic-joystick-drivers ros-melodic-pointcloud-to-laserscan ros-melodic-robot-localization ros-melodic-spacenav-node ros-melodic-tf2-sensor-msgs ros-melodic-twist-mux python3-numpy python3-empy
```

3. Install the SubT Virtual Challenge Software.

    ```
	sudo apt-get install subtsim
	```

4. Open a terminal and run the first tunnel practice environment.

    ```
	subtsim-quickstart
	```
	
5. Use your joystick to move each robot according to the following mapping:

6. Now it's time to move beyond the joystick, and write a solution to the SubT Challange! See the Resource below to get started.

## Resources

1. [Tutorials](https://bitbucket.org/osrf/subt/wiki/tutorials)
1. [Documentation](https://bitbucket.org/osrf/subt/wiki/documentation)
1. [Continuous Integration](https://bitbucket.org/osrf/subt/addon/pipelines/home#!/results/branch/default)