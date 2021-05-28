# Edge540XL
This specifications.md file is a description for the JHU/APL Edge540XL with Sensor Configuration 1. This robot may be launched using a `ign launch` command with the variable name `jhuapl_edge540xl_sensor_config_1`.

## Description
Edge540XL is an acrobatic fixed-wing UAV.
In this configuration Edge540XL is equipped with the following sensors: forward facing color & depth camera.


## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 competition.ign robotName1:=edge540xl robotConfig1:=JHUAPL_EDGE540XL_SENSOR_CONFIG_1  circuit:=cave worldName:=simple_cave_01 localModel:=true
```

At this point you can control the vehicle model with an actuator command, for example:
```
rostopic pub -r 10 /edge540xl/input mav_msgs/Actuators "angular_velocities: [0.0, 0.1, 0.0, 15.0]"
```
This commands the angular velocities of the aileron, elevator, and rudder, with thrust at the end.

## Usage Rights
The software included in this package is released under a BSD 3-Clause license.

### Cost and Scale
The Edge540XL is a research platform and its cost is not publicly available at the moment. It weighs approximately 1.25 kg.

### Sensors
This Edge540XL with sensor configuration 1 includes the following sensors.

* Depth Camera - Intel Realsense D435, modeled by `rgbd_camera` plugin
* IMU
* Gas sensor
* Barometer
* Magnetometer 

### Control
A high-level controller that takes in a goal Pose will be released in the future.

### Motion Characteristics

* _x_ linear velocity range from 3.0 m/s to 15 m/s
* _x_ linear acceleration range from -30.0 m/s<sup>2</sup> to 13 m/s<sup>2</sup>
* control surface velocity range from -10 rad/s to 10 rad/s
* thrust range from 0 N to 16 N

### Endurance Characteristics

* battery capacity: 2.2Ah
* voltage fully charged: 12.6V
* maximum draw: 500W
* average draw: ~200W

### Diversions from Physical Hardware of Edge540XL


## Edge540XL Specification Links
* [Depth Camera - Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/)
