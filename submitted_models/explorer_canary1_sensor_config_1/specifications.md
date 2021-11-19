	<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Explorer - Canary1
This specifications.md file is a description and proof of virtual model validation for
Explorer's robot Canary1 with Sensor Configuration 1. This robot may be launched using
an `ign launch ` command with the variable name `explorer_canary1_sensor_config_1`.

## Description
Canary1 is quadrotor UAV. It is designed for exploring various environments, such as tunnels, caves and so on.

## Cost and Scale
Explorer's Canary1 has an estimated commercial cost of USD 40,000. It weighs approximately 9.25 kg.

## Sensors
Canary1 with sensor configuration 1 includes the following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 2x 3D medium range lidar &mdash; Velodyne-16 and an Ouster OS-0, modeled by `gpu_lidar` plugin.
* 1x IMU f &mdash; Xsense MTI-100, modeled by `imu` plugins.
* 2x uEye fisheye rgb cameras with a total of 360 degrees field of view  &mdash; modeled by 4 cameras in sim since there is no fisheye camera model.

## Control
Canary1 is controlled by the open-source teleop_twist_joy package.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion
constraint characteristics:

* _x_ and _y_ velocity range from -5.5 m/s to 5.5 m/s. _z_ velocity ranges from -1.6 m/s to 1.6 m/s.
* _x_, _y_ acceleration range from -2.1 m/s<sup>2</sup> to 2.1 m/s<sup>2</sup>. _z_ acceleration ranges from -4 m/s<sup>2</sup> to 4 m/s<sup>2</sup>.
* Angular velocity ranges from -0.8 rad/s to 0.8 rad/s.

The constraints can be found in the following locations within the simulation model package:

* `spawner.rb`, lines 113-115

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following
endurance characteristics:

* Battery life of 1020 seconds

## Diversions from Physical Hardware of Canary1
Virtual Canary1 is a faithful representation of real-world Canary1 both in appearance and
in physical properties. However, there are few diversions:
* It does not model long-range Rajant communication of the real robot.
* Since there are no fisheye camera models in the sim, it uses 4 simulated cameras to get the full 360 degree field of view around the simulated robot that the 2 fisheye cameras get for the real robot.

# <a name="validation_links"></a>Canary1 Validation and Specification Links

* https://www.amtechs.co.jp/product/VLP-16-Puck.pdf
* https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p1-os0.pdf?__hstc=34987006.1615e6c550c7c10f70939cc54a917b79.1606276933776.1621366418434.1634865387151.13&__hssc=34987006.1.1634865387151&__hsfp=3056247791
