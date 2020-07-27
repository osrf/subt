	<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Explorer - DS1
This specifications.md file is a description and proof of virtual model validation for
Explorer's robot DS1 with Sensor Configuration 1. This robot may be launched using
a `roslaunch` command with the variable name `explorer_ds1_sensor_config_1`.
## Description
DS1 is quadrotor UAV. It is designed for exploring various environments, such as tunnels, caves and so on.

## Cost and Scale
Explorer's DS1 has an estimated commercial cost of USD 21,000. It weighs approximately 5.2 kg.

## Sensors
DS1 with sensor configuration 1 includes the following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 3x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 1x 3D medium range lidar &mdash; Velodyne-16, modeled by `gpu_lidar` plugin.
* 1x IMU f &mdash; Xsense MTI-100, modeled by `imu` plugins.

## Control
DS1 is controlled by the open-source teleop_twist_joy package.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ and _y_ velocity range from -4.0 m/s to 4.0 m/s. _z_ velocity ranges from -1.5 m/s to 1.5 m/s.
* _x_, _y_ acceleration range from -3.9 m/s<sup>2</sup> to 3.9 m/s<sup>2</sup>. _z_ acceleration ranges from -2 m/s<sup>2</sup> to 2 m/s<sup>2</sup>.
* Angular velocity ranges from -1.2 rad/s to 1.2 rad/s.

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following
endurance characteristics:

* Battery life of 600 seconds

## Diversions from Physical Hardware of DS1
Virtual DS1 is a faithful representation of real-world DS1 both in appearance and
in physical properties. However, there are few diversions:
* It does not model long-range Rajant communication of the real robot.
* It does not model the exact dynamic model (mass and inertial) of real-world DS1, instead it's using the motion parameters of standard X3 UAV model for control perspective. 

# <a name="validation_links"></a>DS1 Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* https://autonomoustuff.com/product/velodyne-puck-vlp-16/
* https://www.mouser.com/datasheet/2/693/mti-100-series-1540263.pdf
* \<Validation Motion Test Link, https://youtu.be/K0IjFDt_e80/\>
* \<Validation Endurance Test Link, https://youtu.be/AeAUZVCKmkA/\>
