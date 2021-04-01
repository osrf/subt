
<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Rocky
This specifications.md file is a description of Coordinated Robotics Rocky with Sensor
Configuration 1. This robot may be launched using a `ign launch` command with the
variable name `coro_rocky_sensor_config_1`.

## Description
Rocky is a four-wheel Ackermann type steering ground vehicle. Rocky weighs ~4 kg.

## Usage Instructions
The robot is controlled by the included Ackermann steering plugin and accepts a
linear x velocity and a rotational z velocity. Sensors are published on the standard
topics.

## Usage Rights


## Cost and Scale
Rocky has an estimated duplication cost of USD 1470. It weighs 4.076 kg.
Cost breakdown:
$400  - computer
$280  - Frame
$270  - Motors / controller
$250  - D435i
$170  - misc
$100  - Wide cameras / lenses


## Sensors
Rocky Sensor Config 1 uses:
* 1 x D435i modeled by rgbd_camera plugin
* 1 x Kakute F7 - modeled by imu and air_pressure plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Arducam B0247 with LN008 - modeled by camera plugins
* 4 x Wheel odometry - wheel encoders modeled by pose-publisher plugin

Rocky Sensor Config 2 adds 6 breadcrumbs
Rocky Sensor Config 3 has a 2D lidar (RPLidar S1), but there is not space for breadcrumbs with the lidar, so no Sensor Config 4



## Control
Rocky is controlled by the AckermannSteering plugin and accepts a linear x velocity
and a rotational z velocity.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -3.0 m/s to 3.0 m/s
* _x_ acceleration range from -3.60 m/s<sup>2</sup> to 6.61 m/s<sup>2</sup> (modeled as +- 4m/s/s)
* Turning radius of 1.60 m


The constraints can be found in the following locations within the simulation model package:

* Turning radius is modeled by the `AckermannSteering` plugin in the `spawner.rb` file, line 24
* Velocity and acceleration limits applied to the `AckermannSteering` plugin in the `spawner.rb` file, lines 28-31


## Endurance Characteristics
This vehicle has a battery life of 10482 seconds.  The 4 cell 16.8V 6.6AH battery started at 16.57V and finished at 16.23V after driving in a circle for 20 minutes.
10482 seconds = 1200 seconds * (16.57V - 13.6V) / (16.57V / 16.23V)


## Diversions from Physical Hardware of Rocky
Fisheye cameras are not modeled.  The two fisheye cameras are modeled as two 1280x720 wide field of view cameras.  They could have been modeled as 4 cameras,
but this robot's sensor platform didn't have four sided symmetry so it seemed to make more sense to just have two cameras on this model.

The differentials are modeled as two separate motors rather than one motor driving two wheels.

Suspension is not modeled.


## Rocky Validation and Specification Links

* Vehicle - https://www.nitrorcx.com/-03c09-madtorque-green-artr.html
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Arducam B0247 - https://www.arducam.com/product/arducam-2mp-ar0230-obisp-mipi-camera-module-for-raspberry-pi-jetson-nano/
* Arducam LN008 - https://www.arducam.com/product/m25156h14/
* D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
* 2D lidar - RPLidar S1 - https://www.slamtec.com/en/Lidar/S1

* Turning Circle -  https://youtu.be/H15Kpvpepv4
* Endurance - https://youtu.be/2A16jvbZ4UI
* Speed / Acceleration Test -  https://youtu.be/XBhK73EbVq4
* Datalogs / datasheets / etc - https://drive.google.com/drive/folders/1y_q9G3xNFaG_KOAiWuYeCj1hfao2BsOg?usp=sharing
