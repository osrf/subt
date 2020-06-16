<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Explorer - R2
This specifications.md file is a description and proof of virtual model validation for
Explorer's robot R2 with Sensor Configuration 1. This robot may be launched using
a `roslaunch` command with the variable name `explorer_r2_sensor_config_1`.
## Description
R2 is four wheels ground robot. It is designed for exploring various environments, such as tunnel, cave and so on.

## Cost and Scale
Explorer's R2 has an estimated commercial cost of USD 7,000. It weighs approximately 200 kg.

## Sensors
Robot R2 with sensor configuration 1 includes the following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 4x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 1x 3D medium range lidar &mdash; Velodyne-16, modeled by `gpu_lidar` plugin.
* 1x IMU f &mdash; LORD Sensing 3DM-GX5, modeled by `imu` plugins.

## Control
R2 is controlled by the open-source teleop_twist_joy package.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -2.0 m/s to 2.0 m/s
* _x_ acceleration range from -10 m/s<sup>2</sup> to 10 m/s<sup>2</sup>
* Turning radius of 0.6 m

The constraints can be found in the following locations within the simulation model
package:

* left front wheel maximum velocity and torque &mdash; model.sdf, lines 626 and 627
* Other wheels &mdash; just below Left front wheel in the same file.

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following
endurance characteristics:

* Battery life of 5400 seconds

## Diversions from Physical Hardware of R2
Virtual R2 is a faithful representation of real-world R2 both in appearance and
in physical properties. However, there are few diversions:
* It does not model long-range low-bandwidth (LoRa) mesh communication of the real robot.

# <a name="validation_links"></a>R2 Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* https://autonomoustuff.com/product/velodyne-puck-vlp-16/
