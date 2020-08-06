<!-- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->

# CERBERUS-Aerial Scout
This specifications.md file is a description and proof of virtual model validation for
the CERBERUS Aerial Scout with Sensor Configuration 1. This robot may be launched using a `roslaunch` command with the variable name `CERBERUS_GAGARIN_SENSOR_CONFIG_1`.

## Description
CERBERUS Gagarin Aerial Scout is a highly autonomous system capable of being deployed in an unknown environment and proceeding for its mapping up to the point of automatic return to home. The platform basis is designed by Flyability but the rest is custom integration and new algorithms.
In this configuration, CERBERUS Aerial Scout is equipped with the following sensors: 
LIDAR, front facing color camera, IMU and upward facing depth sensor.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=gagarin robotConfig1:=CERBERUS_GAGARIN_SENSOR_CONFIG_1 localModel:=true
```
The velocity of the robot can be controlled with following command:
```
rostopic pub /gagarin/cmd_vel geometry_msgs/Twist "linear:
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
The CERBERUS Gagarin robot is a research platform and its cost is not publicly available at the moment.
It weighs approximately 2.316 kg.

### Sensors
This Aerial Scout with sensor configuration 1 includes the following sensors. 
The following specific sensors are declared payloads of this vehicle:
* IMU - Generic, modeled by `imu` plugin
* LIDAR - Ouster OS1-16, modeled by `gpu_ray` plugin
* Color Camera - FLIR Blackfly S, modeled by `camera` plugin
* Depth Sensor - Picoflexx, modeled by `depth_camera` plugin

### Control
This Aerial Scout is controlled by the default twist controller package inside the simulation environment.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following motion constraint characteristics.
* _x_ linear velocity range from -2.0 m/s to 2.0 m/s
* _y_ linear velocity range from -2.0 m/s to 2.0 m/s
* _z_ linear velocity range from -1.0 m/s to 1.0 m/s
* _x_ linear acceleration range from -1.0 m/s<sup>2</sup> to 1 m/s<sup>2</sup>
* _y_ linear acceleration range from -1.0 m/s<sup>2</sup> to 1.0 m/s<sup>2</sup>
* _z_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _x_ angular velocity range from -2.0 rad/s to 2.0 rad/s
* _y_ angular velocity range from -2.0 rad/s to 2.0 rad/s
* _z_ angular velocity range from -3.0 rad/s to 3.0 rad/s
* _x_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20 rad/s<sup>2</sup>
* _y_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20.0 rad/s<sup>2</sup>
* _z_ angular acceleration range from -9.0 rad/s<sup>2</sup> to 9.0 rad/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model
package ignition::gazebo::systems::MulticopterVelocityControl (file src/subt/submitted_models/cerberus_gagarin_sensor_config_1/launch/spawner.rb)
* Linear acceleration - <maximumLinearAcceleration>1 1 4</maximumLinearAcceleration>
* Linear velocity - <maximumLinearVelocity>2 2 1</maximumLinearVelocity>
* Angular velocity - <maximumAngularVelocity>2 2 3</maximumAngularVelocity>

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following endurance characteristics.

* Battery life of about 300 seconds

### Diversions from Physical Hardware of Aerial Scout
* A camera model is used but of course camera sensors are hard to model exactly. 
The field of view and the resolution are the models matched.
* Controller in the simulation model is different from the one in the real platform. The control state vector is different for the real platform. We utilized the default controller for other micro aerial vehicles in the repo and tuned performance to match our closed-loop dynamics.
* The IMU on the simulated platform differs from the one one the real robot.

## <a name="validation_links"></a>Aerial Scout Validation and Specification Links
* Base Platform: Flyability Elios 2 [Link](https://www.flyability.com/elios-2)
* IMU - Generic Analog Devices IMU: [Link](https://www.analog.com/en/parametricsearch/11172#/)
* LIDAR -Ouster OS1-16: [Link](https://ouster.com/products/os1-lidar-sensor/)
* Color Camera - FLIR - Blackfly S USB3 Model ##BFS-U3-16S2C-CS: [Link](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS)
* Depth Sensor - CamBoard Picoflexx: [Link](https://pmdtec.com/picofamily/)
* Linear Acceleration and Speed Validation Video: [Link](https://youtu.be/VifPQmhGkI0)
* Linear Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/182n8QBHvxxJ1CyJELmbLYlBwFnsa7cvz/view?usp=sharing)
* Angular Acceleration and Speed Validation Video: [Link](https://youtu.be/Bn8VlWzRhWA)
* Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1SrO9YOMJKTqt4tkjJ2kLfCnKyU1NIVUV/view?usp=sharing)
* Battery Endurance Validation Video: [Link](https://youtu.be/v8OhcbGmqOU)
* Battery Endurance Validation Data: [Link](https://drive.google.com/file/d/1H3UiX_vYeS0c1cc393jz0ahVzi4LfQmP/view?usp=sharing)
