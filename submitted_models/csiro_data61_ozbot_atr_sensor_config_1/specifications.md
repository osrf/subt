<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# CSIRO_DATA61 OZBOT_ATR Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for
the CSIRO_DATA61 OZBOT ATR with Sensor Configuration 1. This robot may be launched using an `ign launch` command with the
`robotConfigN` variable name `CSIRO_DATA61_OZBOT_ATR_SENSOR_CONFIG_1`.

## Description
This robot configuration is based on the BIA5 OZBOT-ATR Tracked AGV. It has a standardised perception payload mounted at the front of the vehicle, including cameras and a gimbal mounted lidar.

## Usage Instructions
The robot accepts standard twist commands on the cmd_vel topic. The gimbal for the lidar can be controlled on the following topic:
lidar_gimbal/pan_rate_cmd_double (std_msgs/Double)

The position of the gimbal is accessed on the joint state topic. The joint state topic also contains the states of all the other joints on the robot (for the wheels used to simulate track drive), but the gimbal state is the first joint on the topic, e.g joint_states/position[0]

## Usage Rights
This software is released under a [BSD 3-Clause license](LICENSE).

### Cost and Scale
The BIA5 OZBOT-ATR platform used by CSIRO_DATA61 is a custom design and so does not have a publicly available cost or specifications. The total robot weight is approximately 150Kg.

### Sensors
The CSIRO_DATA61 OZBOT-ATR with sensor configuration 1, includes the following sensors. The specifications for these instruments are provided below in
the [Validation Links](#markdown-header-ozbot-atr-validation-and-specification-links)) section.
The following specific sensors are declared payloads of this vehicle:

* A Velodyne VLP-16 Lidar, modeled by the gpu_lidar plugin. (note this lidar is also mounted at 45 degrees on a rotating gimbal for to give a near-360 degree FOV)
* ECON e-CAM130_CUXVR Quad Camera system with each camera mounted on one side of the payload. The version on the platform is a custom set of sensors run at a resolution of 2016x1512, hardware triggered at 15fps. They are modeled by the standard camera plugin
* A Microstrain CV5-25 IMU, modeled by the standard imu plugin
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

### Control
The platform is controlled by the standard diff-drive plugin for ignition gazebo, with twist inputs on "/<robotName>/cmd_vel", The robot has the following motion characteristics. Note there is also a gimbal for spinning the lidar, which is controlled on "/"<robotName>/lidar_gimbal/pan_rate_cmd_double"

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf),
this vehicle has the following motion constraint characteristics.

* linear (x) velocity range from -1.2 m/s to 1.2 m/s
* angular(z) velocity range from -2.6 rad/s to 2.6 rad/s

* linear (x) acceleration range from -1.0 m/s<sup>2</sup> to 1.0 m/s<sup>2</sup>
* Angular (z) acceleration range from -2.6 rad/s<sup>2</sup> to 2.6 rad/s<sup>2</sup>

* Turning radius of 0m, can turn on the spot

### Endurance Characteristics
Endurance of the platform is approximately 312 minutes

### Diversions from Physical Hardware of OZBOT_ATR
* Currently the model does not have an accurate render of the track system, as diff-drive control currently requires the use of simulated multi-wheel systems. The multi-wheel system on the sim model is intended to provide a reasonable approximation of the track contact geometry for driving.
* The physical version of the robot features a complex suspension system for the tracks, this is not feasible to model in the simulation and thus in simulation the robot is completely rigid.

## OZBOT ATR Validation and Specification Links
* [BIA5](https://bia5.com/)
* [Validation video](https://youtu.be/U0o8-M2kiaE)
* [Validation Data](https://drive.google.com/file/d/1DWd2lvQOATFQ2wIntZEg7R6HxGG7bK9k/view?usp=sharing)
* [Endurance Validation video](https://youtu.be/vYFuMpeoP5k)
* [Endurance Validation Data](https://drive.google.com/file/d/12rqR-LBI3oEzU-ufZDIpkwxkszsmju1Q/view?usp=sharing)
* [LIDAR - Velodyne VLP-16](https://velodynelidar.com/products/puck/)
* [IMU - Microstrain CV5-25](https://www.microstrain.com/inertial/3dm-cv5-25)
* [Cameras - ECON e-CAM130_CUXVR](https://www.e-consystems.com/nvidia-cameras/jetson-agx-xavier-cameras/four-synchronized-4k-cameras.asp)
