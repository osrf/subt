<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Robotika - Kloubak
This specifications.md file is a description and proof of virtual model validation for
Robotika's robot Kloubak with Sensor Configuration 1. This robot may be launched using
a `roslaunch` command with the variable name `robotika_kloubak_sensor_config_1`.
## Description
Klubak is four wheels articulated robot. It is designed for research tasks in agriculture e.g. plants monitoring and treatment.

## Cost and Scale
Robotika's Kloubak has an estimated commercial cost of USD 7,000. It weighs approximately 30 kg.

## Sensors

The following specific sensors are declared payloads of this vehicle:

* 2x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 2x Planar lidar &mdash; SICK TIM881P, modeled by `gpu_lidar` plugin.
* 2x Monocular camera &mdash; Arecont Vision AV3216DN  + lens MPL 1.55, modeled by `camera` plugin.
* 2x IMU f &mdash; LORD Sensing 3DM-GX5, modeled by `imu` plugins.
* Altimeter &mdash; Infineon DPS310, modeled by `air_pressure` plugin.
* Wheel odometry - wheel encoders modelled by `pose-publisher` plugin.

The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

## Control
Kloubak is controlled by the open-source Osgar framework.

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -2.0 m/s to 2.0 m/s
* _x_ acceleration range from -9.8 m/s<sup>2</sup> to 9.8 m/s<sup>2</sup>
* Turning radius of 0.8 m

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following
endurance characteristics:

* Battery life of 5400 seconds

## Diversions from Physical Hardware of Kloubak
Virtual Kloubak is a faithful representation of real-world Kloubak both in appearance and
in physical properties. However, there are few diversions:

* It does not model long-range low-bandwidth (LoRa) mesh communication of the real robot.
* It does not model realsense tracking cameras t265.
* It does not model angle sensor in the main joint and the second IMU is used instead.

# <a name="validation_links"></a>Kloubak Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* Nearest public documentation to TIM881P: https://www.sick.com/ch/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim581-2050101/p/p619344
* https://sales.arecontvision.com/product/MegaVideo+G5/AV3216DN
* https://www.microstrain.com/sites/default/files/applications/files/3dm-gx5-10_datasheet_8400-0095_rev_j.pdf
* https://www.infineon.com/cms/en/product/sensor/pressure-sensors/absolute-pressure-sensors-map-bap/dps310/
* https://heltec.org/project/wifi-lora-32/
