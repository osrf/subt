<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Bandicoot
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Bandicoot with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_bandicoot_sensor_config_1`.

## Description
Bandicoot is a quadrotor UAV with a 3D LIDAR and cameras.  Bandicoot weighs 3.278KG.  Bandicoot is based on the Pam type UAV with significant modifications to reduce weight and better position the 3D LIDAR.

## Usage Instructions
The robot can be used in the same by sending Twist commands to the vehicle_name/cmd_vel ROS topic.


## Usage Rights


## Cost and Scale
Bandicoot has an estimated duplication cost of USD 11310. It weighs 3.278KG.
Cost breakdown:
$8000 - Ouster OS0-64 LIDAR
$1300 - MS-IMU3025
$540  - Frame / motors / props / batteries / controllers
$420  - lights / connectors / DC-DC / misc
$400  - 2 x D435i
$400  - Computer
$150  - LIDAR-Lite V3HP x 1
$100  - wide cameras / lenses

## Sensors
Bandicoot Sensor Config 1 uses:
* 2 x D435i modeled by rgbd_camera plugin
* 1 x MS-IMU3025 - modeled by imu_sensor and magnetometer plugin
* 1 x Kakute F7 - modeled by air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 1 x Garmin LIDAR-Lite v3HP modeled by gpu_ray

## Control
Bandicoot is controlled by by the Twist ROS topic cmd_vel.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:


* _x_ velocity range from -6.5 m/s to 6.5 m/s
* _x_ acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _y_ velocity range from -6.0 m/s to 6.0 m/s
* _y_ acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _z_ velocity range from -3.0 m/s to 3.0 m/s
* _z_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>
* _z_ velocity range from -1.9 rad/s to 1.9 rad/s


The constraints can be found in the following locations within the simulation model package:
* Velocity and acceleration limits applied to the `MulticopterVelocityControl` plugin in the `spawner.rb` file, lines 113-115


## Endurance Characteristics
The vehicle has a battery life of 20:45


## Diversions from Physical Hardware of Bandicoot



## Bandicoot Validation and Specification Links
* Garmin LIDAR-Lite v3HP - https://buy.garmin.com/en-US/US/p/578152
* Ouster OS0-64 LiDAR - https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p0-os0.pdf
* Computer - https://developer.nvidia.com/embedded/jetson-xavier-nx-devkit
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* Arducam B0203 - https://www.arducam.com/product/arducam-2mp-ar0230-obisp-mipi-camera-module-for-raspberry-pi-jetson-nano/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/

Validation data uploaded to Box.
