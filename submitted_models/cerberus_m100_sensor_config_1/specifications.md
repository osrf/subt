<!-- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->

# CERBERUS-Aerial Scout
This specifications.md file is a description and proof of virtual model validation for
the CERBERUS Aerial Scout with Sensor Configuration 1. This robot may be launched using a `roslaunch` command with the variable name `CERBERUS_M100_SENSOR_CONFIG_1`.

## Description
CERBERUS Aerial Scout is highly autonomous system capable of being deployed in an unknown environment and proceeding for its mapping up to the point of automatic return to home. The platform basis is an M100 but the rest is custom integration and new algorithms.
In this configuration, CERBERUS Aerial Scout is equipped with the following sensors: LIDAR, donward facing color camera and an IMU.

## Usage Instructions
The vehicle can be launched with the following command:
```
ign launch -v 4 src/subt/submitted_models/cerberus_m100_sensor_config_1/launch/m100_test.ign robotName:=m100 modelName:=CERBERUS_M100_SENSOR_CONFIG_1 localModel:=true
```
The velocity of the robot can be controlled with following command:
```
rostopic pub /m100/cmd_vel geometry_msgs/Twist "linear:
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
The CERBERUS Aerial Scout robot has an estimated cost of $20,000. It weighs approximately 3.759 kg.

### Sensors
This Aerial Scout with sensor configuration 1 includes the
following sensors. 
The following specific sensors are declared payloads of this vehicle:
* IMU - Vector Nav VN100, modeled by `imu` plugin
* LIDAR - Velodyne VLP-16, modeled by `gpu_ray` plugin
* Color Camera - FLIR Blackfly S, modeled by `camera` plugin

### Control
This Aerial Scout is controlled by the default twist controller package inside the simulation environment.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics.
* _x_ linear velocity range from -3.0 m/s to 3.0 m/s
* _y_ linear velocity range from -3.0 m/s to 3.0 m/s
* _z_ linear velocity range from -1.8 m/s to 1.8 m/s
* _x_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4 m/s<sup>2</sup>
* _y_ linear acceleration range from -4.0 m/s<sup>2</sup> to 4.0 m/s<sup>2</sup>
* _z_ linear acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _x_ angular velocity range from -3.0 rad/s to 3.0 rad/s
* _y_ angular velocity range from -3.0 rad/s to 3.0 rad/s
* _z_ angular velocity range from -1.9 rad/s to 1.9 rad/s
* _x_ angular acceleration range from -15.0 rad/s<sup>2</sup> to 15 rad/s<sup>2</sup>
* _y_ angular acceleration range from -15.0 rad/s<sup>2</sup> to 15.0 rad/s<sup>2</sup>
* _z_ angular acceleration range from -5.0 rad/s<sup>2</sup> to 5.0 rad/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model
package ignition::gazebo::systems::MulticopterVelocityControl (file src/subt/submitted_models/cerberus_m100_sensor_config_1/launch/spawner.rb)
* Linear acceleration - <maximumLinearAcceleration>4 4 5</maximumLinearAcceleration>
* Linear velocity - <maximumLinearVelocity>3 3 1.8</maximumLinearVelocity>
* Angular velocity - <maximumAngularVelocity>3 3 1.9</maximumAngularVelocity>

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following endurance characteristics.

* Battery life of 500 seconds

### Diversions from Physical Hardware of Aerial Scout
* A Velodyne PuckLite is used (no second version for OUSTER OS-1)
* A camera model is used but of course camera sensors are hard to model exactly. The field of view and the resolution are the models matched.
* Controller in the simulation model is different from the one in the real platform. The control state vector is different for the real platform. We utilized the default controller for other micro aerial vehicles in the repo and tuned performance to match our closed-loop dynamics.
* FLIR Tau 2 core not present in simulation package (optional configuration)

## <a name="validation_links"></a>Aerial Scout Validation and Specification Links
* Vehicle Specification: [Link](https://www.dronenerds.com/products/drones/enterprise-drones/matrice/dji-matrix-100-flying-platform-matrix100-dji.html?utm_source=google&utm_medium=cpc&adpos=&scid=scplpCP.TP.000029&sc_intid=CP.TP.000029&utm_campaign=SC+Shopping+-+Branded+Desktop&utm_term=&utm_source=adwords&utm_medium=ppc&hsa_net=adwords&hsa_kw=&hsa_tgt=pla-885414664776&hsa_grp=54370605725&hsa_ver=3&hsa_cam=1423409008&hsa_acc=7518285824&hsa_ad=273821055577&hsa_mt=&hsa_src=g&gclid=CjwKCAjwguzzBRBiEiwAgU0FT7dBsblXMt6sIPeH5G7IeGc9V9QWlztFg_4RoNxdS4h0n5HrtsBULRoCh7QQAvD_BwE&utm_source=LS&utm_medium=je6NUbpObpQ&utm_campaign=VigLink&ranMID=42561&ranEAID=je6NUbpObpQ&ranSiteID=je6NUbpObpQ-4aX1pU9kM_R9TMrhILzx5Q) 
* IMU - VectorNav VN-100 IMU: [Link](https://www.vectornav.com/products/vn-100)
* LIDAR - Velodyne - VLP-16 PuckLite: [Link](https://velodynelidar.com/products/puck/)
* Color Camera - FLIR - Blackfly S USB3 Model ##BFS-U3-16S2C-CS: [Link](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS)  
* Linear Acceleration and Speed Validation Video: [Link](https://youtu.be/GtoS-pPHVWM)
* Linear Acceleration and Speed Validation Data: [Link](https://drive.google.com/open?id=14V-pT-VASC975NN3iFFi9GCIuhRQ-Abo)
* Angular Acceleration and Speed Validation Video: [Link](https://youtu.be/yrCuiDCqyC4)
* Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/open?id=1fvhH8tl3zbtQOTJdGs-8LPccpg8fS7XP)
* Battery Endurance Validation Video: [Link](https://youtu.be/VKiZHleWNq4)
* Battery Endurance Validation Data: [Link](https://drive.google.com/open?id=12tBNsxiN895O99cq8roC0xSOMllQV-cG)
