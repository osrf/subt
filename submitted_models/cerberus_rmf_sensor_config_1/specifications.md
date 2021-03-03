<!-- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->

# CERBERUS-Aerial Scout
This specifications.md file is a description and proof of virtual model validation for
the CERBERUS Aerial Scout with Sensor Configuration 1. This robot may be launched using a launch command with the variable name `CERBERUS_RMF_SENSOR_CONFIG_1`.

## Description
CERBERUS RMF (Resilient Micro Flyer) Aerial Scout is a highly autonomous system capable of being deployed in an unknown environment and proceeding for its mapping up to the point of automatic return to home. The platform basis is designed by Flyability but the rest is custom integration and new algorithms.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=rmf robotConfig1:=CERBERUS_RMF_SENSOR_CONFIG_1 localModel:=true
```
The velocity of the robot can be controlled with following command:
```
rostopic pub /rmf/cmd_vel geometry_msgs/Twist "linear:
  x: -0.1
  y: 0.1
  z: 0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```

## Usage Rights
The software included in this package is released under a [BSD 3-Clause license](LICENSE).


### Cost and Scale
The CERBERUS RMF robot is a research platform and its cost is not publicly available at the moment.
It weighs approximately 0.524 kg. The estimated cost to build one physical unit is approximately US Dollars 400.

### Sensors
This Aerial Scout with sensor configuration 1 includes the following sensors. 
The following specific sensors are declared payloads of this vehicle:
* IMU - Generic, modeled by `imu` plugin
* 4x ToF Ranging sensor - VL53L0X, modeled by `gpu_ray` plugin
* Tracking Camera - T265, modeled by two `camera` sensors (right and left) with large FoV.

### Control
This Aerial Scout is controlled by the default twist controller package inside the simulation environment.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following motion constraint characteristics.
* _x_ linear velocity range from -3.8 m/s to 3.8 m/s
* _y_ linear velocity range from -3.8 m/s to 3.8 m/s
* _z_ linear velocity range from -1.6 m/s to 2.6 m/s
* _x_ linear acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>
* _y_ linear acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>
* _z_ linear acceleration range from -5.5 m/s<sup>2</sup> to 5.5 m/s<sup>2</sup>
* _x_ angular velocity range from -3.9 rad/s to 3.9 rad/s
* _y_ angular velocity range from -3.7 rad/s to 3.7 rad/s
* _z_ angular velocity range from -3.4 rad/s to 3.4 rad/s
* _x_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20 rad/s<sup>2</sup>
* _y_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20.0 rad/s<sup>2</sup>
* _z_ angular acceleration range from -9.0 rad/s<sup>2</sup> to 9.0 rad/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model
package ignition::gazebo::systems::MulticopterVelocityControl (file src/subt/submitted_models/cerberus_rmf_sensor_config_1/launch/spawner.rb)
* Linear acceleration - <maximumLinearAcceleration>3 3 5.5</maximumLinearAcceleration>
* Linear velocity - <maximumLinearVelocity>3.8 3.8 2.6</maximumLinearVelocity>
* Angular velocity - <maximumAngularVelocity>3.9 3.7 3.4</maximumAngularVelocity>

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following endurance characteristics.

* Battery life of about 600 seconds

### Diversions from Physical Hardware of Aerial Scout
* Two camera sensors with wide FoV are used to model the T265 sensor, however the simulation model does not match the real model
as the real model contains Fisheye lens with a large FoV. The FoV has been reduced to 2.8 radians from 3.02 radians. And the images in simulation are having RGB8 colour format as compared to L8 format on the real hardware.
* Controller in the simulation model is different from the one in the real platform. The control state vector is different for the real platform. We utilized the default controller for other micro aerial vehicles in the repo and tuned performance to match our closed-loop dynamics.
* The IMU on the simulated platform differs from the one one the real robot.

## <a name="validation_links"></a>Aerial Scout Validation and Specification Links
* Base Platform: Autonomous Robots Lab - Resilient Micro Flyer
* IMU - Generic Analog Devices IMU: [Link](https://www.analog.com/en/parametricsearch/11172#/)
* ToF Range Sensor -VL53L0X: [Link](https://www.pololu.com/product/2490)
* Tracking Camera - Realsense T265 : [Link](https://www.intelrealsense.com/tracking-camera-t265/)
* Linear Acceleration and Speed Validation Video: [Link](https://drive.google.com/file/d/1hep_ti0LY3wNCooUpJh06TTT2gG3RPNn/view?usp=sharing)
* Linear Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1I8nKsqnE6CqWwJkut4WVpc1BPDECNTLL/view?usp=sharing)
* Angular Acceleration and Speed Validation Video: [Link](https://drive.google.com/file/d/1wEF2rkPWH0N_U9Yr5QXQZMPTysrr9y6B/view?usp=sharing)
* Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1sLb_djgq5xFi3PKDqrje2H1CVppl2KKp/view?usp=sharing)
* Battery Endurance Validation Video: [Link](https://drive.google.com/file/d/1Qzbuuz_tl8gtKxU1OBH5mrgaueKcTPmB/view?usp=sharing)
* Battery Endurance Validation Data: [Link](https://drive.google.com/file/d/19VTE4STCADNO1WHdsoeUbEBw_41VDj3y/view?usp=sharing)
