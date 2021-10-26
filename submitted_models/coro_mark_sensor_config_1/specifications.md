<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Mark
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Mark with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_mark_sensor_config_1`.

## Description
Mark is a quadrotor UAV with a D435i cameras.  Mark weighs 2.265KG.

## Usage Instructions
The robot can be used by sending Twist commands to the vehicle_name/cmd_vel ROS topic.


## Usage Rights


## Cost and Scale
Mark has an estimated duplication cost of USD 1310. It weighs 2.265KG.
Cost breakdown:
$370 - Frame / motors / props / batteries / controllers
$280 -  Computer
$260 - LIDAR-Lite v3 x 2
$200  - D435i
$150 - lights / connectors /switches / DC-DC / misc


## Sensors
Mark Sensor Config 1 uses:
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Kakute F7 - modeled by air_pressure plugin, imu_sensor
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Garmin LIDAR-Lite v3 modeled by gpu_ray
* 1 x magnetometer - The real vehicle doesn't have a magnetometer, but is in place to be consistent with all the other drones.

## Control
Mark is controlled by by the Twist ROS topic cmd_vel.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -7.8 m/s to 7.6 m/s
* _x_ acceleration range from -5.7 m/s<sup>2</sup> to 4.8 m/s<sup>2</sup>
* _y_ velocity range from -5.5 m/s to 6.2 m/s
* _y_ acceleration range from -5.1 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _z_ velocity range from -4.8 m/s to 6.1 m/s
* _z_ acceleration range from -9.6 m/s<sup>2</sup> to 5.1 m/s<sup>2</sup>
* _z_ velocity range from -3.3 rad/s to 3.1 rad/s
* _z_ acceleration range from -5.0 rad/s<sup>2</sup> to 4.2 rad/s<sup>2</sup>


The constraints can be found in the following locations within the simulation model package:
* Velocity and acceleration limits applied to the `MulticopterVelocityControl` plugin in the `spawner.rb` file, lines 113-115


## Endurance Characteristics
This vehicle has a battery life of 15 minutes.



## Diversions from Physical Hardware of Mark



## Mark Validation and Specification Links
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Garmin LIDAR-Lite v3HP - https://buy.garmin.com/en-US/US/p/557294
