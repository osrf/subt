<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Explorer - R3
This specifications.md file is a description and proof of virtual model validation for
Explorer's robot R3 with Sensor Configuration 1. This robot may be launched using
an `ign launch` command with the variable name `explorer_r3_sensor_config_1`.

## Description
R3 is four wheels ground robot. It is designed for exploring various environments, such as tunnel, cave and so on.

## Cost and Scale
Explorer's R3 has an estimated commercial cost of USD 100000. It weighs approximately 99 kg.

## Sensors
Robot R3 with sensor configuration 1 includes the following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 4x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 1x 3D medium range lidar &mdash; Velodyne-16, modeled by `gpu_lidar` plugin.
* 1x IMU &mdash; Xsense MTI-100, modeled by `imu` plugins.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

## Control
R3 is controlled by the open-source teleop_twist_joy package.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -1.0 m/s to 1.0 m/s
* _x_ acceleration range from -1.24 m/s<sup>2</sup> to 1.16 m/s<sup>2</sup>
* Angular velocity ranges from -0.95 rad/s to 0.99 rad/s.
* Turning radius of 0.0 m (turns on the spot)
* Rotation angle between two parts of the robot is limited to -30-30 degree
* Spring reference is set to 0.1 and spring stiffness is set to 100 for joint Center Pivot

The constraints can be found in the following locations within the simulation model
package:

* Left wheel spring coefficients &mdash; model.sdf, line 636 and 637
* Other wheels &mdash; just below Left front wheel in the same file.
* Rotation angle limit &mdash; model.sdf, line 553 to 556
* Spring coefficients &mdash; model.sdf, line 559 and 560
* `diff_drive` controller limits in `spawner.rb`, lines 23-26

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following
endurance characteristics:

* Battery life of ~~5400 seconds~~, currently modeled as 1 hour until validation data can be provided 

## Diversions from Physical Hardware of R3
Virtual R3 is a faithful representation of real-world R3 both in appearance and
in physical properties. However, there are few diversions:
* The endurance is approximated to match existing models until model validation data can be provided.
* It does not model long-range Rajant wireless network of the real robot.
* The physical robot has 9 communication breadcrumbs. 12 breadcrumbs are included in sensor configuration 2, which is standardized to match other available models.

# <a name="validation_links"></a>R3 Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
* https://www.mouser.com/datasheet/2/693/mti_100_series-1540263.pdf
* \<Validation Motion Test Data Link, https://drive.google.com/file/d/1kyLJ4BiAJMeZhl9Pf2bQzIq-8Aa5PB0b/view\>
* \<Validation Motion Test Video Link, https://youtu.be/0dcYIWql4q4\>

