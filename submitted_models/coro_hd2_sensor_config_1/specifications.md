<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# Coordinated Robotics HD2 Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the Coordinated Robotics HD2 with Sensor Configuration 1. This robot may be launched using an ign launch command with the variable name `coro_hd2_sensor_config_1`.

## Description
This configuration is based on Superdroid HD2 ground robot. The HD2 weighs ~40KG.  A drone can be carried atop the sensor platform.  It is based on Team MARBLE's HD2 model.

## Usage Instructions
The robot is controlled by the skid steer plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.

## Usage Rights


### Cost and Scale
This has an estimated duplication cost of USD 11675.
Cost breakdown:
$5700 - Superdroid HD2
$2500 - Velodyne VLP-16 https://www.ebay.com/sch/i.html?_nkw=velodyne+vlp-16&_sacat=0
$1300 - MS-IMU3025
$900  - lights / connectors /switches / DC-DC / misc
$800  - Computer 
$250  - D435i
$225  - Seek Thermal Compact

### Sensors
Sensor Config 1 uses:
* 1 x Velodyne VLP-16, modeled by gpu_lidar plugin
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Memsense MS-IMU3025 modeled by imu_sensor and magnetometer plugin
* 1 x Kakute F7 - modeled by air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Arducam B0203 with LN008 - modeled by 4 camera plugins
* 1 x Seek Thermal Compact - modeled by the thermal plugin

Sensor Config 2 adds 12 breadcrumbs:
* 12 x Raspberry Pi Zero W

### Control
This HD2 is controlled by the TrackedVehicle plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_ velocity range from -1.0 m/s to 1.0 m/s
* _x_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model package:

* `spawner.rb`, lines 27-30

The motion and endurance characteristics have not been validated yet.  It is expected the characteristics will be roughly similar to the default ones.


### Endurance Characteristics
Endurance set to 1 hour until validation testing is completed.


### Diversions from Physical Hardware
Fisheye cameras are not modeled, so 4 cameras were used for simulation in place of the two fisheye cameras on the physical robot.


## Validation and Specification Links
* Superdroid HD2 - https://www.superdroidrobots.com/shop/item.aspx/configurable-hd2-treaded-atr-tank-robot-platform/789/
* VLP-16 LIDAR -  https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
* Memsense MS-IMU3025 - https://www.memsense.com/products/ms-imu3025
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0203 - https://www.arducam.com/product/arducam-1080p-hd-wide-angle-wdr-usb-camera-module-for-computer-2mp-1-2-7-cmos-ar0230-100-degree-mini-uvc-usb2-0-spy-webcam-board-with-3-3ft-1m-cable-for-windows-linux-mac-os-android/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* Seek Thermal Compact - https://www.thermal.com/uploads/1/0/1/3/101388544/compact-sellsheet-usa_web.pdf
