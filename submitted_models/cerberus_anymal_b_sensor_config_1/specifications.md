# ANYmal
This specifications.md file is a description and proof of virtual model validation for the CERBERUS ANYmal B with Sensor Configuration 1. This robot may be launched using a `ign launch` command with the variable name `cerberus_anymal_b_sensor_config_1`.

## Description
ANYmal is a highly sophisticated four-legged robot, designed to tackle the challenges of harsh search and rescue operations, inspections, and other surveillance duties.
In this configuration ANYmal is equipped with the following sensors: LIDAR, downward facing depth camera, forward facing color camera, two color cameras on the sides, and an IMU.

## Dependencies
To be able to control this robot model you need a few additional packages. Please clone the repository [`cerberus_anymal_locomotion`](https://github.com/leggedrobotics/cerberus_anymal_locomotion) and follow the instructions specified in its Readme.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 competition.ign robotName1:=anymal_b robotConfig1:=CERBERUS_ANYMAL_B_SENSOR_CONFIG_1
```

In another terminal you can source the "anymal_locomotion_ws" workspace (see [Dependencies](#markdown-header-dependencies)) to start the locomotion controller node:
```
cd ~/anymal_locomotion_ws/
source devel/setup.bash
roslaunch cerberus_anymal_b_control_1 cerberus_anymal_controller.launch
```

At this point you can control the vehicle model with a twist command, for example:
```
rostopic pub /anymal_b/cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Note that the vehicle moves with a constant forward and angular velocity. This is a design choice and cannot be changed. Therefore the input twist is used only to determine in which direction the vehicle should move. See [Control](#markdown-header-control) for more information.

## Usage Rights
The software included in this package is released under a [BSD 3-Clause license](LICENSE).

ANYmal model is adapted from [anymal_b_simple_description](https://github.com/ANYbotics/anymal_b_simple_description) repository, released under BSD 3-Clause license.

### Cost and Scale
The CERBERUS ANYmal robot is a research platform and its cost is not publicly available at the moment. It weighs approximately 37 kg.

### Sensors
This ANYmal with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#markdown-header-anymal-validation-and-specification-links) section.

* IMU - Xsens MTi 100, modeled by `imu` plugin
* LIDAR - Velodyne VLP-16, modeled by `gpu_lidar` plugin
* LIDAR - Robosense RS-Bpearl, there is not (yet) an Ignition-Gazebo plugin
* Depth Camera - Intel Realsense D435, modeled by `rgbd_camera` plugin
* Color Camera - FLIR Blackfly S Model ##BFS-U3-16S2C-CS , modeled by `camera` plugin
* Synchronization Board - Autonomous Systems Lab, ETH Zurich - VersaVIS, there is not (yet) an Ignition-Gazebo plugin
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

### Control
This ANYmal is controlled by the custom `cerberus_anymal_b_control_1` package, available in the repository [cerberus_anymal_locomotion](https://github.com/leggedrobotics/cerberus_anymal_locomotion).

This controller makes the robot moving at a fixed velocity, in any direction. The input twist is used only to determine the orientation of movement. The bearing angle between X-axis and the orientation specified by the linear part of the twist vector indicates the direction of motion; The positive or negative value of the yaw rate of the twist vector sets the direction of rotation around the Z-axis.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics.

*  _x_ velocity range from -0.45 m/s to 0.45 m/s
*  _y_ velocity range from -0.45 m/s to 0.45 m/s
*  _z_ velocity range from 0 m/s to 0 m/s
*  _x_ acceleration range from -0.024 m/(s*s) to 0.024 m/(s*s)
*  _y_ acceleration range from -0.024 m/(s*s) to 0.024 m/(s*s)
*  _z_ acceleration range from 0 m/(s*s) to 0 m/(s*s)
*  Turning radius of 0 m (can rotate on the spot)

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge, this vehicle has the following endurance characteristics.

* Battery life of 4200 seconds

### Diversions from Physical Hardware of ANYmal
The following points highlight the differences between the submitted virtual model and its hardware counterpart.

* Robosense RS-Bpearl LIDAR is not present in simulation since its Ignition-Gazebo plugin does not exist yet. A downward Intel Realsense is used as a replacement in simulation.
* VersaVIS board is used to time synchronize a camera with an IMU for Visual-Inertial Odometry purposes. At the moment there is no plan to develop an Ignition-Gazebo plugin for that and therefore this board is not included in simulation.

* The physical robot carries 4 communication breadcrumbs. 12 breadcrumbs are included in sensor configuration 2, which is standardized to match other available models.

## ANYmal Validation and Specification Links
* [ANYmal specification link](https://researchfeatures.com/2018/05/01/anymal-unique-quadruped-robot-conquering-harsh-environments/#)
* [ANYbotics](https://www.anybotics.com/)
* [Real World telemetry data](https://drive.google.com/a/leggedrobotics.com/file/d/1G75RJV-w46AjxpkTj-5-CL9r0ISU71eR/view?usp=sharing)
* [Real World telemetry data video](https://www.youtube.com/watch?v=U3EWpmW8xK0&feature=youtu.be)
* [ANYmal weigth](https://drive.google.com/open?id=1h8tA9FCksqqvsR4cqZzJIPo9Vst_VwPs)
* [IMU - Xsens MTi 100](https://www.xsens.com/products/mti-100-series)
* [LIDAR - Velodyne VLP-16](https://velodynelidar.com/products/puck/)
* [LIDAR - Robosense RS-Bpearl](https://www.robosense.ai/rslidar/rs-bpearl)
* [Depth Camera - Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/)
* [Color Camera - FLIR Blackfly S](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS)
* [Synchronization Board - VersaVIS](https://www.mdpi.com/1424-8220/20/5/1439)
