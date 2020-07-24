<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Explorer - R2
This specifications.md file is a description and proof of virtual model validation for
Explorer's robot R2 with Sensor Configuration 1. This robot may be launched using
a `roslaunch` command with the variable name `explorer_r2_sensor_config_1`.
## Description
R2 is four wheels ground robot. It is designed for exploring various environments, such as tunnel, cave and so on.

## Cost and Scale
Explorer's R2 has an estimated commercial cost of USD 100000. It weighs approximately 200 kg.

## Sensors
Robot R2 with sensor configuration 1 includes the following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 4x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 1x 3D medium range lidar &mdash; Velodyne-16, modeled by `gpu_lidar` plugin.
* 1x IMU &mdash; Xsense MTI-100, modeled by `imu` plugins.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

## Control
R2 is controlled by the open-source teleop_twist_joy package.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from ~~-2.0 m/s to 2.0 m/s~~, currently modeled as -1.0 m/s to 1.0 m/s until validation data can be provided
* _x_ acceleration range from -10 m/s<sup>2</sup> to 10 m/s<sup>2</sup>
* Turning radius of 0.6 m
* Rotation angle between two parts of the robot is limited to -30-30 degree
* Spring reference is set to 0.1 and spring stiffness is set to 100 for joint Center Pivot

The constraints can be found in the following locations within the simulation model
package:

* left front wheel maximum velocity and torque &mdash; model.sdf, lines 626 and 627
* Other wheels &mdash; just below Left front wheel in the same file.
* Rotation angle limit &mdash; model.sdf, line 547 to 550
* Spring coefficients &mdash; model.sdf, line 554 and 555
* `diff_drive` controller limits in `spawner.rb`, lines 23-26

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following
endurance characteristics:

* Battery life of ~~5400 seconds~~, currently modeled as 1 hour until validation data can be provided 

## Diversions from Physical Hardware of R2
Virtual R2 is a faithful representation of real-world R2 both in appearance and
in physical properties. However, there are few diversions:
* The endurance and motion characteristics above are approximated to match existing models until model validation data can be provided.
* It does not model long-range Rajant wireless network of the real robot.
* The physical robot has 9 communication breadcrumbs. 12 breadcrumbs are included in sensor configuration 2, which is standardized to match other available models.

# <a name="validation_links"></a>R2 Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* https://autonomoustuff.com/product/velodyne-puck-vlp-16/
* https://www.mouser.com/datasheet/2/693/mti-100-series-1540263.pdf
