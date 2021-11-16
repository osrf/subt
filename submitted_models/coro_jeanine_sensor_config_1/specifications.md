<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Jeanine
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Jeanine with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_jeanine_sensor_config_1`.

## Description
Jeanine is a four-wheel skid steer ground vehicle. Jeanine weighs ~45 kg and uses a motor
for each wheel.  Jeanine can carry and launch a drone from atop the sensor platform.

## Usage Instructions
The robot is controlled by the skid steer plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.

## Usage Rights


## Cost and Scale
Jeanine has an estimated duplication cost of USD 5850. It weighs  46.6 kg.
Cost breakdown:
$2500 - Velodyne VLP-16 https://www.ebay.com/sch/i.html?_nkw=velodyne+vlp-16&_sacat=0
$1300 - MS-IMU3025
$750 - Frame / motors / batteries / controllers
$700 - lights / connectors /switches / DC-DC / misc
$400 - Computer
$200  - D435


## Sensors
Jeanine Sensor Config 1 uses:
* 1 x Velodyne VLP-16, modeled by gpu_lidar plugin
* 1 x D435 modeled by rgbd_camera plugin
* 1 x Memsense MS-IMU3025 modeled by imu_sensor and magnetometer plugin
* 1 x Kakute F7 - modeled by air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Arducam B0203 with LN008 - modeled by 4 camera plugins


Jeanine Sensor Config 2 adds 12 breadcrumbs:
* 12 x Raspberry Pi Zero W

## Control
Jeanine is controlled by the DiffDrive plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

Velocity limited to 1.0m/S until validation is completed.


The constraints can be found in the following locations within the simulation model package:

* Front left wheel maximum velocity and torque &mdash; model.sdf, lines 1415 and 1417
* Other wheels &mdash; just below Front Left wheel in the same file.
* Velocity and acceleration limits applied to the `diff_drive` plugin in the `spawner.rb` file, lines 23-26

## Endurance Characteristics
Battery limited to 1 hour until validation is completed.

## Diversions from Physical Hardware of Jeanine
Fisheye cameras are not modeled, so 4 cameras were used for simulation in place of the two fisheye cameras on the physical robot.

## Jeanine Validation and Specification Links

* VLP-16 LIDAR -  https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
* Memsense MS-IMU3025 - https://www.memsense.com/products/ms-imu3025
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0203 - https://www.arducam.com/product/arducam-1080p-hd-wide-angle-wdr-usb-camera-module-for-computer-2mp-1-2-7-cmos-ar0230-100-degree-mini-uvc-usb2-0-spy-webcam-board-with-3-3ft-1m-cable-for-windows-linux-mac-os-android/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* D435 RGBD Camera - https://www.intelrealsense.com/depth-camera-d435/
