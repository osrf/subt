
<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Karen
This specifications.md file is a description of Coordinated Robotics Karen with Sensor
Configuration 1. This robot may be launched using a `ign launch` command with the
variable name `coro_karen_sensor_config_1`.

## Description
Karen is a four-wheel Ackermann type steering ground vehicle. Karen weighs ~110 kg.
Karen can carry and launch a drone from atop the sensor platform.
Karen is currently in the lab at CSUCI and so is not accessible for validation
testing.  Velocity, weight, battery life and steering radius are taken from
the manufacturer's web page.

## Usage Instructions
The robot is controlled by the included Ackermann steering plugin and accepts a
linear x velocity and a rotational z velocity. Sensors are published on the standard
topics.

## Usage Rights


## Cost and Scale
Karen has an estimated duplication cost of USD 9,975. It weighs 110 kg.
Cost breakdown:
$3500 - Frame / motor / gearboxes / batteries / controllers
$2500 - Velodyne VLP-16 https://www.ebay.com/sch/i.html?_nkw=velodyne+vlp-16&_sacat=0
$1300 - MS-IMU3025
$1200 - lights / connectors /switches / DC-DC / misc
$1000 - laptop
$250  - D435i
$225  - Seek Thermal Compact
The original frame / motor / gearbox was purchased from craigslist for $1200.  We added
batteries and a controller for $300 more.  I would estimate it would cost $3200 to
purchase the parts necessary to build a mechanically similar robot.  The manufacturer
leases a complete system with sensors, software, support, etc. for $1500/month.

## Sensors
Karen Sensor Config 1 uses:
* 1 x Velodyne VLP-16, modeled by gpu_lidar plugin
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Memsense MS-IMU3025 modeled by imu_sensor and magnetometer plugin
* 1 x Kakute F7 - modeled by air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Arducam B0203 with LN008 - modeled by 4 camera plugins
* 1 x Seek Thermal Compact - modeled by the thermal plugin
* 4 x Wheel odometry - wheel encoders modeled by pose-publisher plugin

Karen Sensor Config 2 adds 12 breadcrumbs:
* 12 x Raspberry Pi Zero W

## Control
Karen is controlled by the AckermannSteering plugin and accepts a linear x velocity
and a rotational z velocity.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -1.67 m/s to 1.67 m/s
* _x_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>
* Turning radius of 5.0 m

The velocity range is based on the manufacturer's speed of 6km/hour.
Acceleration is estimated.
Turning radius is based on the manufacturer's specification.

The constraints can be found in the following locations within the simulation model package:

* Turning radius is modeled by the `AckermannSteering` plugin in the `spawner.rb` file, line 24
* Velocity and acceleration limits applied to the `AckermannSteering` plugin in the `spawner.rb` file, lines 28-31
* Front left wheel maximum velocity and torque &mdash; model.sdf, lines 1295-1296
* Other wheels &mdash; just below Front Left wheel in the same file.


## Endurance Characteristics
This vehicle has a battery life of 14400 seconds.  This is based on the manufacturers range of 24km
divided by a speed of 6km/hour.  The batteries are two large 12V deep cycle batteries.


## Diversions from Physical Hardware of Karen
Fisheye cameras are not modeled, so 4 cameras were used for simulation in place of the two fisheye cameras on the physical robot.

The rear differential is modeled as two separate motors rather than one motor driving two wheels.

The front wheels are driven in the simulator, but not on the physical vehicle.  This is so that the vehicle
will make it up the slopes in the simulator.  It may also be possible to adjust the wheel friction
parameters and only drive the rear wheels.  The manufacturer does list a 15 degree slope climbing angle
and the simulator slopes are ~30 degrees.  Based on the weight balance, motor torque and pneumatic wheels
I believe the physical vehicle can go to 40 degrees or so, but that has not been validated.


## Karen Validation and Specification Links

* Vehicle - https://smprobotics.com/products_autonomous_ugv/
* VLP-16 LIDAR -  https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
* Memsense MS-IMU3025 - https://www.memsense.com/products/ms-imu3025
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0203 - https://www.arducam.com/product/arducam-1080p-hd-wide-angle-wdr-usb-camera-module-for-computer-2mp-1-2-7-cmos-ar0230-100-degree-mini-uvc-usb2-0-spy-webcam-board-with-3-3ft-1m-cable-for-windows-linux-mac-os-android/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* Seek Thermal Compact - https://www.thermal.com/uploads/1/0/1/3/101388544/compact-sellsheet-usa_web.pdf
