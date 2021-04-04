<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Coordinated Robotics - Pam
This specifications.md file is a description and proof of virtual model validation for
Coordinated Robotics Pam with Sensor Configuration 1. This robot may be launched using
a `ign launch` command with the variable name `coro_pam_sensor_config_1`.

## Description
Pam is a quadrotor UAV with 3 D435 cameras.  Pam weighs 3.6KG.

## Usage Instructions
The robot can be used in the same by sending Twist commands to the vehicle_name/cmd_vel ROS topic.


## Usage Rights


## Cost and Scale
Pam has an estimated duplication cost of USD 2160. It weighs 3557 grams.
Cost breakdown:
$600  - 3 x D435
$540 - Frame / motors / props / batteries / controllers
$500 -  Computer - GB-BRi5-8250
$300 - LIDAR-Lite V3HP x 2
$220 - lights / connectors /switches / DC-DC / misc


## Sensors
Pam Sensor Config 1 uses:
* 3 x D435 modeled by rgbd_camera plugin
* 1 x Kakute F7 - modeled by air_pressure plugin, imu_sensor, and magnetometer plugin
* 1 x SCD30 - modeled by GasEmitterDetector plugin
* 2 x Garmin LIDAR-Lite v3HP modeled by gpu_ray

## Control
Pam is controlled by by the Twist ROS topic cmd_vel.  Sensors are published on the standard topics.


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -5.5 m/s to 7.6 m/s
* _x_ acceleration range from -5.3 m/s<sup>2</sup> to 5.2 m/s<sup>2</sup>
* _y_ velocity range from -4.9 m/s to 5.3 m/s
* _y_ acceleration range from -5.1 m/s<sup>2</sup> to 4.6 m/s<sup>2</sup>
* _z_ velocity range from -3.4 m/s to 3.7 m/s
* _z_ acceleration range from -3.6 m/s<sup>2</sup> to 4.1 m/s<sup>2</sup>
* _z_ velocity range from -2.5 rad/s to 3.4 rad/s
* _z_ acceleration range from -5.2 rad/s<sup>2</sup> to 4.4 rad/s<sup>2</sup>


The constraints can be found in the following locations within the simulation model package:
* Velocity and acceleration limits applied to the `MulticopterVelocityControl` plugin in the `spawner.rb` file, lines 113-115


## Endurance Characteristics
This vehicle has a battery life of 1150 seconds.

## Diversions from Physical Hardware of Pam
Intel Realsense Tracking Camera T265 not modeled.

# <a name="validation_links"></a>Pam Validation and Specification Links

* D435 RGBD Camera - https://www.intelrealsense.com/depth-camera-d435/
* Kakute F7 - http://www.holybro.com/product/kakute-f7-v1-5/
* Brix GB-BRi5-8250 - http://download.gigabyte.cn/FileList/Manual/brix_kabylake_datasheet.pdf
* Garmin LIDAR-Lite v3HP - https://buy.garmin.com/en-US/US/p/578152
* https://youtu.be/fMVmpcDZjTY  (endurance test)
* https://youtu.be/S93qIGnCB9o  (xy speed, rotation test)
* https://youtu.be/IJbUCzJX2yM  (z speed test)
* https://drive.google.com/drive/folders/1RYP9wEd9TDOLieNbupwiiLK58UEyGYs8?usp=sharing (datalogs, etc.)
