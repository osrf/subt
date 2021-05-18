<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Crystal
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Crystal with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_crystal_sensor_config_1`.

## Description
Crystal is a quadrotor UAV with 3 D435 cameras.  Crystal weighs 1.5KG.

## Usage Instructions
The robot can be used in the same by sending Twist commands to the vehicle_name/cmd_vel ROS topic.


## Usage Rights


## Cost and Scale
Crystal has an estimated duplication cost of USD 935. It weighs 1.5KG.
Cost breakdown:
$200 - D435i
$150 - lights / connectors / DC-DC / propellors / misc
$130 - LIDAR-Lite v3
$120 - Jetson Nano
$100 - motors - https://www.getfpv.com/t-motor-f40-pro-iii-motor.html
$65  - ESC - https://www.getfpv.com/t-motor-f45a-v2-3-6s-blheli-32-4-in-1-esc.html
$50  - Wide camera / lens
$50  - flight controller - https://www.getfpv.com/holybro-kakute-f7-v1-5-flight-controller.html
$40  - Battery - https://hobbyking.com/en_us/turnigy-battery-4000mah-4s-30c-lipo-pack-xt-60.html
$30  - frame - https://www.racedayquads.com/collections/rdq-source-one-frames-spare-parts/products/rdq-source-one-v3-7-long-range-frame-6mm-v0-2-arms


## Sensors
Crystal Sensor Config 1 uses:
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Arducam B0247 with LN008 - modeled by camera plugin
* 1 x Kakute F7 - modeled by air_pressure plugin, imu_sensor
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 1 x Garmin LIDAR-Lite v3 modeled by gpu_ray
* 1 x magnetometer - The real vehicle doesn't have a magnetometer, but is in place to be consistent with all the other drones.

## Control
Crystal is controlled by by the Twist ROS topic cmd_vel.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

Limited to default 4m/s as tests are not completed.


The constraints can be found in the following locations within the simulation model package:
* Velocity and acceleration limits applied to the `MulticopterVelocityControl` plugin in the `spawner.rb` file, lines 113-115


## Endurance Characteristics
Limited to default 10 minutes as tests are not completed.

## Diversions from Physical Hardware of Crystal


## Crystal Validation and Specification Links
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0247 - https://www.arducam.com/product/arducam-2mp-ar0230-obisp-mipi-camera-module-for-raspberry-pi-jetson-nano/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* Garmin LIDAR-Lite v3 - https://buy.garmin.com/en-US/US/p/557294
