<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# ANYmal
This specifications.md file is a description and proof of virtual model validation for the CERBERUS ANYmal C with Sensor Configuration 1. This robot may be launched using a `ign launch` command with the variable name `cerberus_anymal_c_sensor_config_1`.

## Description
ANYmal is a highly sophisticated four-legged robot, designed to tackle the challenges of harsh search and rescue operations, inspections, and other surveillance duties.
In this configuration ANYmal is equipped with the following sensors: LIDAR, two downward facing half-sphere LIDARs, IMU and a perception unit including seven cameras and an additional IMU.

## Dependencies
To be able to control this robot model you need a few additional packages. Please clone the repository [`cerberus_anymal_locomotion`](https://github.com/leggedrobotics/cerberus_anymal_locomotion) and follow the instructions specified in its README.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 competition.ign robotName1:=anymal_c robotConfig1:=CERBERUS_ANYMAL_C_SENSOR_CONFIG_1
```

In another terminal you can source the "anymal_locomotion_ws" workspace (see [Dependencies](#markdown-header-dependencies)) to start the locomotion controller node:
```
cd ~/anymal_locomotion_ws/
source devel/setup.bash
roslaunch cerberus_anymal_c_control_1 cerberus_anymal_controller.launch
```

At this point you can control the vehicle model with a twist command, for example:
```
rostopic pub /anymal_c/cmd_vel geometry_msgs/Twist "linear:
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

ANYmal model is adapted from [anymal_c_simple_description](https://github.com/ANYbotics/anymal_c_simple_description) repository, released under BSD 3-Clause license.

### Cost and Scale
The CERBERUS ANYmal robot is a research platform and its cost is not publicly available at the moment. It weighs approximately 55 kg.

### Sensors
This ANYmal with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#markdown-header-anymal-validation-and-specification-links) section.

* Onboard IMU - N/A, modeled by `imu` plugin
* LIDAR - Velodyne VLP-16, modeled by `gpu_lidar` plugin
* LIDAR - Robosense RS-Bpearl, modeled by `gpu_lidar` plugin
* VIO Perception Head - Alphasense, modeled by `imu` and `camera` plugin
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

### Control
This ANYmal is controlled by the custom `cerberus_anymal_c_control_1` package, available in the repository [cerberus_anymal_locomotion](https://github.com/leggedrobotics/cerberus_anymal_locomotion).

This controller makes the robot moving at a fixed velocity, in any direction. The input twist is used only to determine the orientation of movement. The bearing angle between X-axis and the orientation specified by the linear part of the twist vector indicates the direction of motion; The positive or negative value of the yaw rate of the twist vector sets the direction of rotation around the Z-axis.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics.

*  _x_ velocity range from -0.709 m/s to 0.704 m/s
*  _y_ velocity range from -0.716 m/s to 0.715 m/s
*  _z_ velocity range from 0 m/s to 0 m/s
*  _x_ acceleration range from -0.756 m/(s*s) to 0.903 m/(s*s)
*  _y_ acceleration range from -0.736 m/(s*s) to 0.953 m/(s*s)
*  _z_ acceleration range from 0 m/(s*s) to 0 m/(s*s)
*  Turning radius of 0 m (can rotate on the spot)

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following endurance characteristics.

* Battery life of 4269 seconds

### Diversions from Physical Hardware of ANYmal
The following points highlight the differences between the submitted virtual model and its hardware counterpart.

* The perception head features 3 gray scale and 4 color cameras. These cameras are modeled as color cameras only.
* The physical robot carries 4 communication breadcrumbs. 12 breadcrumbs are included in sensor configuration 2, which is standardized to match other available models.

## ANYmal Validation and Specification Links
* [ANYmal specification link](https://researchfeatures.com/2018/05/01/anymal-unique-quadruped-robot-conquering-harsh-environments/#)
* [ANYbotics](https://www.anybotics.com/)
* [Real World telemetry data/video](https://drive.google.com/drive/folders/1DlhimLBMShALMtkspDM-va2piQx8fRKq?usp=sharing)
* [ANYmal weigth](https://drive.google.com/file/d/1zJPQx23A_2rpEeE7_yEe7X13pZEPaJBG/view?usp=sharing)
* [LIDAR - Velodyne VLP-16](https://velodynelidar.com/products/puck/)
* [LIDAR - Robosense RS-Bpearl](https://www.robosense.ai/rslidar/rs-bpearl)
* [VIO Perception Head - Alphasense](https://github.com/sevensense-robotics/alphasense_core_manual)
