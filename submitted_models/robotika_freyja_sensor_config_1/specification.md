<!--- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->
# Robotika - Freyja
This specifications.md file is a description and proof of virtual model validation for
Robotika's robot Freyja with Sensor Configuration 1. This robot may be launched using
a `roslaunch` command with the variable name `robotika_freyja_sensor_config_1`.
## Description
Freyja is a four-wheel all-terrain ground vehicle. With weight under 30 kg and its
medium size, Freyja can be transported by a single person as a check-in baggage on an
airplane.

## Cost and Scale
Robotika's Freyja has an estimated commercial cost of USD 7,000. It weighs approximately
29.6  kg.

## Sensors
Freyja in Sensor Configuration 1 is equipped with forward-looking and backward looking
medium-fps RGB & Depth cameras with wide field of view (Mynte Eye D), four even
wider-field-of-view color cameras looking all around (KAYETON), two planar lidars
(SICK TIM881P), and with other sensors, such as imu combined with a magnetic compass
(CMPS14), an altimeter (Infineon DPS310), a CO2 sensor (MH-Z19B) and in-wheel encoders.

The specifications for these instruments are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 2x RGBD camera &mdash; Mynt Eye D, modeled by `rgbd_camera` plugin.
* 2x Planar lidar &mdash; SICK TIM881P, modeled by `gpu_lidar` plugin.
* 4x Monocular camera &mdash; KAYETON, modeled by `camera` plugin.
* Tilt-compensated compass &mdash; CMPS14, modeled by `imu` and `magnetometer`
plugins.
* Altimeter &mdash; Infineon DPS310, modeled by `air_pressure` plugin.
* Wheel odometry - wheel encoders modelled by `pose-publisher`
plugin.
* Gas sensor &mdash; Winsen MH-Z19B, modeled by GasEmitterDetector plugin.
## Control
Freyja is controlled by the open-source Osgar framework or by custom Erro framework.
## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion
constraint characteristics:

* _x_ velocity range from -2.5 m/s to 2.5 m/s
* _x_ acceleration range from -10.6 m/s<sup>2</sup> to 10.6 m/s<sup>2</sup>
* Turning radius of 0.0 m

The constraints can be found in the following locations within the simulation model
package:

* Front left wheel maximum velocity and torque &mdash; model.sdf, lines 1415 and 1417
* Other wheels &mdash; just below Front Left wheel in the same file.

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following
endurance characteristics:

* Battery life of 14400 seconds

## Diversions from Physical Hardware of Freyja
Virtual Freyja is a faithful representation of real-world Freya both in appearance and
in physical properties. It does not model long-range low-bandwidth (LoRa) mesh
communication of the real robot.

# <a name="validation_links"></a>Freyja Validation and Specification Links

* https://www.uumotor.com/10-inch-single-shaft-brushless-motor-sensorless.html
* https://www.robot-electronics.co.uk/cmps14-tilt-compensated-compass.html
* Nearest public documentation to TIM881P: https://www.sick.com/ch/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim581-2050101/p/p619344
* https://www.kayetoncctv.com/global-shutter-high-speed-120fps-hd-720p-webcam-uvc-plug-play-driverless-usb-camera-module-for-android-linux-windows-mac/
* https://www.infineon.com/cms/en/product/sensor/pressure-sensors/absolute-pressure-sensors-map-bap/dps310/
* https://www.winsen-sensor.com/sensors/co2-sensor/mh-z19b.html
* https://youtu.be/cHJ-ZloPmhE (speed test)
* https://youtu.be/bZ83Zneuhf8 (rotation test)
* https://drive.google.com/drive/folders/1M8W3vjP1G0Pz7oeB7dAkBjXhYE05hvxA?usp=sharing
* https://heltec.org/project/wifi-lora-32/
