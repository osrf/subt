<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Allie
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Allie with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_allie_sensor_config_1`.

## Description
Allie is a four-wheel skid steer ground vehicle. Allie weighs ~50 kg and uses a motor
for each wheel.  Allie can carry and launch a drone from atop the sensor platform.

## Usage Instructions
The robot is controlled by the skid steer plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.

## Usage Rights


## Cost and Scale
Allie has an estimated duplication cost of USD 8,600. It weighs 47.1 kg.
Cost breakdown:
$2500 - Velodyne VLP-16 https://www.ebay.com/sch/i.html?_nkw=velodyne+vlp-16&_sacat=0
$2350 - Frame / motors / gearboxes / batteries / controllers
$1300 - MS-IMU3025
$1200 - lights / connectors /switches / DC-DC / misc
$1000 - Dell G3 laptop
$250  - D435i

## Sensors
Allie Sensor Config 1 uses:
* 1 x Velodyne VLP-16, modeled by gpu_lidar plugin
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Memsense MS-IMU3025 modeled by imu_sensor and magnetometer plugin
* 1 x Kakute F7 - modeled by air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Arducam B0203 with LN008 - modeled by 4 camera plugins
* 4 x Wheel odometry - wheel encoders modeled by pose-publisher plugin

Allie Sensor Config 2 adds 12 breadcrumbs:
* 12 x Raspberry Pi Zero W

## Control
Allie is controlled by the DiffDrive plugin and accepts a linear x velocity and a rotational z velocity.
Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -2.8 m/s to 2.8 m/s
* _x_ acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _z_ velocity range from -4.0 rad/s to 4.0 rad/s
* _z_ acceleration range from -12.4 rad/s<sup>2</sup> to 12.4 rad/s<sup>2</sup>
* Turning radius of 0.0 m

The constraints can be found in the following locations within the simulation model package:

* Front left wheel maximum velocity and torque &mdash; model.sdf, lines 1415 and 1417
* Other wheels &mdash; just below Front Left wheel in the same file.
* Velocity and acceleration limits applied to the `diff_drive` plugin in the `spawner.rb` file, lines 23-26

## Endurance Characteristics
This vehicle has a battery life of 9000 seconds.  The 4 cell 16.8V 40AH battery started at 16.785V and finished at 16.392V after driving in a circle for 20 minutes.
9725 seconds = 1200 seconds * (16.785V - 13.6V) / (16.785V / 16.392V)
I rounded down to 9000 seconds as that was the calculated result from endurance test v1.

## Diversions from Physical Hardware of Allie
Fisheye cameras are not modeled, so 4 cameras were used for simulation in place of the two fisheye cameras on the physical robot.

# <a name="validation_links"></a>Allie Validation and Specification Links

* VLP-16 LIDAR -  https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
* Memsense MS-IMU3025 - https://www.memsense.com/products/ms-imu3025
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0203 - https://www.arducam.com/product/arducam-1080p-hd-wide-angle-wdr-usb-camera-module-for-computer-2mp-1-2-7-cmos-ar0230-100-degree-mini-uvc-usb2-0-spy-webcam-board-with-3-3ft-1m-cable-for-windows-linux-mac-os-android/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* https://youtu.be/uEtft8CetpA (speed test)
* https://youtu.be/6BbGLp9J9W0 (rotation test)
* https://youtu.be/taEptt-JU4c (endurance test v2)
* https://drive.google.com/drive/folders/15PRCiCWjE2RYVUxbbYfNrXB4JZoPsjwk?usp=sharing (datalogs, etc.)
