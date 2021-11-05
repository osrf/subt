<!-- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->

# CERBERUS-Aerial Scout
This specifications.md file is a description and proof of virtual model validation for
the CERBERUS Aerial Scout with Sensor Configuration 1. This robot may be launched using a launch command with the variable name `CERBERUS_RMF_OBELIX_SENSOR_CONFIG_1`.

## Description
CERBERUS RMF OBELIX (Resilient Micro Flyer Obelix) Aerial Scout is a highly autonomous system capable of being deployed in an unknown environment and proceeding for its mapping up to the point of automatic return to home. The platform basis is custom designed as so are the algorithms and the integratation of the onboard sensors.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=rmf_obelix robotConfig1:=CERBERUS_RMF_OBELIX_SENSOR_CONFIG_1 localModel:=true
```
The velocity of the robot can be controlled with following command:
```
rostopic pub /rmf_obelix/cmd_vel geometry_msgs/Twist "linear:
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
The CERBERUS RMF Obelix robot is a research platform and its cost is not publicly available at the moment.
It weighs approximately 1.3 kg. The estimated cost to build one physical unit is approximately US Dollars 400.

### Sensors
This Aerial Scout with sensor configuration 1 includes the following sensors.
The following specific sensors are declared payloads of this vehicle:
* IMU - Generic, modeled by `imu` plugin.
* LIDAR - Ouster OS0, modeled by `gpu_ray` plugin.
* Color Camera - FLIR Blackfly S, modeled by `camera` plugin.

### Control
This Aerial Scout is controlled by the default twist controller package inside the simulation environment.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following motion constraint characteristics.
* _x_ linear velocity range from -2.5 m/s to 2.5 m/s
* _y_ linear velocity range from -2.5 m/s to 2.5 m/s
* _z_ linear velocity range from -2.0 m/s to 2.0 m/s
* _x_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _y_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _z_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _x_ angular velocity range from -2.25 rad/s to 2.25 rad/s
* _y_ angular velocity range from -2.25 rad/s to 2.25 rad/s
* _z_ angular velocity range from -1.5 rad/s to 1.5 rad/s
* _x_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20 rad/s<sup>2</sup>
* _y_ angular acceleration range from -20.0 rad/s<sup>2</sup> to 20.0 rad/s<sup>2</sup>
* _z_ angular acceleration range from -6.0 rad/s<sup>2</sup> to 6.0 rad/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model
package ignition::gazebo::systems::MulticopterVelocityControl (file src/subt/submitted_models/cerberus_rmf_obelix_sensor_config_1/launch/spawner.rb)
* Linear acceleration - <maximumLinearAcceleration>4.0 4.0 4.0</maximumLinearAcceleration>
* Linear velocity - <maximumLinearVelocity>2.5 2.5 2.0</maximumLinearVelocity>
* Angular velocity - <maximumAngularVelocity>2.25 2.25 1.5</maximumAngularVelocity>

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following endurance characteristics.

* Battery life of about 500 seconds

### Diversions from Physical Hardware of Aerial Scout
* Controller in the simulation model is different from the one in the real platform. The control state vector is different for the real platform. We utilized the default controller for other micro aerial vehicles in the repo and tuned performance to match our closed-loop dynamics.
* The IMU on the simulated platform differs from the one one the real robot.

## <a name="validation_links"></a>Aerial Scout Validation and Specification Links
* Base Platform: Autonomous Robots Lab - Resilient Micro Flyer Obelix
* IMU - Generic Analog Devices IMU: [Link](https://store.mrobotics.io/mRo-PixRacer-Pro-p/m10064c.htm)
* 3D Lidar Ouster OS0-64: [Link](https://ouster.com/products/os0-lidar-sensor/)
* RGB Camera - FLIR Blackfly S USB3 Model ##BFS-U3-16S2C-CS: [Camera](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS) [Lens](https://www.m12lenses.com/2-1mm-F1-8-Mega-Pixel-Board-Lens-p/pt-02118bmp.htm)
* Linear Acceleration and Speed Validation Video: [Link](https://youtu.be/SKuDyiVHIew)
* Linear Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1tufNXwWsiig21aGcyZbx9EFLVhFCNDA8/view?usp=sharing)
* Angular Acceleration and Speed Validation Video: [Link](https://youtu.be/y3MnuECQexQ)
* Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1x2mcY8x8hj65spy33pKnM4Nze2FW4fRJ/view?usp=sharing)
* Battery Endurance Validation Video: [Link](https://youtu.be/6OKgpclp4aQ)
* Battery Endurance Validation Data: [Link](https://drive.google.com/file/d/1nXiXdnsnQFK1RXZJdVLrYBZh1Q2lIPMz/view?usp=sharing)
