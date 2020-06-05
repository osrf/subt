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
Robot Kloubak with sensor configuration 1 includesthe following sensors. The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 2x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 2x Planar lidar &mdash; SICK TIM881P, modeled by `gpu_lidar` plugin.
* 2x Monocular camera &mdash; Arecont Vision AV3216DN  + lens MPL 1.55, modeled by `camera` plugin.
* 2x IMU f &mdash; LORD Sensing 3DM-GX5, modeled by `imu` plugins.
* Altimeter &mdash; Infineon DPS310, modeled by `air_pressure` plugin.
* Wheel odometry - wheel encoders modelled by `pose-publisher` plugin.

## Control
Kloubak is controlled by the open-source Osgar framework. The robot Kloubak consists of the front and rear differential 
drives connected by joint.

### 4WD drive mode
For control in this mode the angle between front and rear axles is required. The speeds for individual wheels for movement in a circle can be calculated as follow:

_v_fl_ = _v_rl_ = _v_ / _r_ * (_r_ - WHEEL_DISTANCE / 2)

_v_fr_ = _v_rr_ = _v_ / _r_ * (_r_ + WHEEL_DISTANCE / 2)

where _v_fl_, _v_rl_, _v_fr_ and _v_rr_ are speeds for individual wheels front left, rear left, front right and rear right respectively.
_v_ is the desired speed of the robot centroid and _r_ is the current turning radius. Calculation of the current turning radius:

_r_ = _L_ / _tan_(_&phi;_ /2)

where _L_ is the distance between axle center and robot joint and _&phi;_ is the angle between front and rear axles. 
For  change of the turning radius, three separate movements should be considered. 
Movement in a circle (described above), rotation of individual axles and axle displacement (towards or away from each other).


## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -2.5 m/s to 2.5 m/s
* _x_ acceleration range from -8.8 m/s<sup>2</sup> to 8.8 m/s<sup>2</sup>
* Turning radius of 0.51 m (distance between axle center and center of  movement)

The constraints can be found in the following locations within the simulation model
package:

* Front left wheel maximum velocity and torque &mdash; model.sdf, lines 1015 and 1017
* Other wheels &mdash; just below Front Left wheel in the same file.

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
* Real robot can be controlled in a "FRONT" mode as well. In this mode only front or rear wheels are driven. Other wheels are passive.

# <a name="validation_links"></a>Kloubak Validation and Specification Links

* https://www.intelrealsense.com/depth-camera-d435/
* Nearest public documentation to TIM881P: https://www.sick.com/ch/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim581-2050101/p/p619344
* https://sales.arecontvision.com/product/MegaVideo+G5/AV3216DN
* https://www.microstrain.com/sites/default/files/applications/files/3dm-gx5-10_datasheet_8400-0095_rev_j.pdf
* https://www.infineon.com/cms/en/product/sensor/pressure-sensors/absolute-pressure-sensors-map-bap/dps310/
* https://heltec.org/project/wifi-lora-32/
* Validation data: https://owncloud.cesnet.cz/index.php/s/0SVMsyiaIwQ9V99
* Speed test video: https://www.youtube.com/watch?v=98LH0aPeuq8
* Angular speed and turning radius limits: https://www.youtube.com/watch?v=CKyTvVXpdSM
