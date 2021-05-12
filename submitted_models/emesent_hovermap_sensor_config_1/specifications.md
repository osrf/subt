<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# Emesent Hovermap Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the Emesent Hovermap with Sensor Configuration 1. This robot may be launched using an `ign launch` command with the variable name `EMESENT_HOVERMAP_SENSOR_CONFIG_1`.

## Description
The Emesent Hovermap is an autonomous system capable of being deployed in an unknown environment. It has a standardised perception payload mounted on the bottom of the vehicle including a gimbal mounted lidar and a gimbal mounted camera system.

## Usage Instructions
The robot accepts standard twist commands on the `cmd_vel` topic.

The gimbal for the lidar can be controlled on the following topic:
* `lidar_gimbal/pan_rate_cmd_double` (std_msgs/Double)

The gimbal controlling camera pose can be controlled on the following topic where each component of the angualr velocity is used to control the roll, pitch and yaw respectively:
* `gimbal/cmd_vel` (geometry_msgs/Twist)

The vehicle can be launched with the following command:
```
ign launch -v 4 competition.ign worldName:=simple_cave_01 circuit:=cave robotName1:=HM robotConfig1:=EMESENT_HOVERMAP_SENSOR_CONFIG_1 localModel:=true
```

The velocity of the robot can be controlled with following command:
```
rostopic pub /HM/cmd_vel geometry_msgs/Twist "linear:
  x: -0.1
  y: 0.1
  z: 0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```
To arm and control the gimbal use the following commands: (Only needs to be armed once)
```
rostopic pub /HM/gimbal/enable std_msgs/Bool true 
rostopic pub /HM/gimbal/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.1
  z: 0.1"
```

## Usage Rights
This software is released under a [BSD 3-Clause license](LICENSE).

### Cost and Scale
The Emesent Hovermap has the following estimate commercial cost components:
* Base vehicle: $12,000
* Sensor suite: $15,000
* Total: ~ $27,000

Its weight is approximately 5.36 kg. 

### Sensors
This Emesent Hovermap with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:
* IMU - Microstrain 3DM-CV5-25 AHRS, modeled by `imu` plugin
* LIDAR - Velodyne VLP-16, modeled by `gpu_lidar` plugin
* IMU - BaseCam IMU Rev.B, modeled by `imu` plugin
* Color Camera - FLIR Blackfly S, modeled by `camera` plugin

### Control
This Emesent Hoverpmap is controlled by the MulticopterVelocityControl plugin.  It accepts twist inputs which drive the vehicle in the simulation environment.  

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_ linear velocity range from -5.0 m/s to 5.0 m/s
* _y_ linear velocity range from -5.0 m/s to 5.0 m/s
* _z_ linear velocity range from -3.0 m/s to 3.0 m/s
* _x_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _y_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _z_ linear acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _x_ angular velocity range from -3.0 rad/s to 3.0 rad/s
* _y_ angular velocity range from -3.0 rad/s to 3.0 rad/s
* _z_ angular velocity range from -1.75 rad/s to 1.75 rad/s
* _x_ angular acceleration range from -15.0 rad/s<sup>2</sup> to 15 rad/s<sup>2</sup>
* _y_ angular acceleration range from -15.0 rad/s<sup>2</sup> to 15.0 rad/s<sup>2</sup>
* _z_ angular acceleration range from -5.0 rad/s<sup>2</sup> to 5.0 rad/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model
package ignition::gazebo::systems::MulticopterVelocityControl (file src/subt/submitted_models/emesent_hovermap_sensor_config_1/launch/spawner.rb)
* Linear acceleration - <maximumLinearAcceleration>4 4 5</maximumLinearAcceleration>
* Linear velocity - <maximumLinearVelocity>5 5 2.8</maximumLinearVelocity>
* Angular velocity - <maximumAngularVelocity>3 3 2.1</maximumAngularVelocity>

We were unable to perform additional validation tests due to access restrictions.

### Endurance Characteristics
Based on the tests specified in the DARPA SubTChallenge [Model Preparation Guide](https://subtchallenge.comresources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following endurance characteristics. 
* Battery life of 1200 seconds 

### Diversions from Physical Hardware of Emesent Hovermap


## Validation and Specification Links
* [Navi Drone]()
* [Hovermap](https://www.emesent.io/hovermap/)
* [Validation Video](https://youtu.be/xxxxxxxxx/)
* [Validation Data](https://drive.google.com/file/xxxxxxxxx/)

* Sensor Specification Links:
  * [LIDAR - Velodyne PuckLITE](https://velodynelidar.com/products/puck-lite/)
  * [IMU - MicroStrain 3DM-CV5-25 AHRS](https://www.microstrain.com/inertial/3dm-cv5-25)
  * [Gimbal IMU - Basecam I2C IMU Rev.B](https://www.basecamelectronics.com/i2c_imu/)
  * [Camera - FLIR BFS-U3-88S6C-C](https://www.flir.com.au/products/blackfly-s-usb3/?model=BFS-U3-88S6C-C)
