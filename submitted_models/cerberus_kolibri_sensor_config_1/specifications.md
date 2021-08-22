<!-- This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# CERBERUS-Kolibri
This specifications.md file is a description and proof of virtual model validation for the CERBERUS Kolibri heavy omnidirectional tri-copter with Sensor Configuration 1. This robot may be launched using a `roslaunch` command with the variable name `CERBERUS_KOLIBRI_SENSOR_CONFIG_1`.

## Description
The Kolibri is a heavy autonomous omnidirectional tricopter for mapping with 6 cameras and an OS-0 lidar. The platform is a customized version of a Voliro - a novel third-party OMAV, that we customized with a state-of-the-art autonomy stack.

## Usage Instructions
Install the SubT workspace according to https://github.com/osrf/subt/wiki/Catkin%20System%20Setup

The vehicle can be launched with the following command:
```
ign launch -v 4 competition.ign circuit:=tunnel worldName:=tunnel_circuit_practice_01 robotName1:=kolibri robotConfig1:=CERBERUS_KOLIBRI_SENSOR_CONFIG_1 localModel:=true enableGroundTruth:=true
```
The velocity of the robot can be controlled with following command:
```
rostopic pub -r 10 /kolibri/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'

```

## Usage Rights
The software included in this package is released under a [BSD 3-Clause license](LICENSE).


### Cost and Mass
The Kolibri has an estimated cost of $30,000. 

It weighs approximately 5.7kg. [Link](https://drive.google.com/file/d/1aj7BvCyJDGn6sZN3biW6XQO5a408WAZe/view?usp=sharing)

### Sensors
This Kolibri with sensor configuration 1 includes the following sensors. 

The following specific sensors are declared payloads of this vehicle:
* IMU - Bosch BMI085, modeled by `imu` plugin
* LIDAR - Ouster OS0, modeled by `gpu_ray` plugin
* B&W/Color Cameras - Sony IMX287, modeled by `camera` plugin

### Control
This Kolibri is controlled by a twist controller package inside the simulation environment.

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics.
* _x_ linear velocity 0.53 m/s (*)
* _y_ linear velocity 0.46 m/s (*)
* _z_ linear velocity 0.46 m/s (*)
* _x_ linear acceleration 0.96 m/s<sup>2</sup>
* _y_ linear acceleration 1.07 m/s<sup>2</sup>
* _z_ linear acceleration 3.5 m/s<sup>2</sup>
* _x_ angular velocity 0 rad/s (**)
* _y_ angular velocity 0 rad/s (**)
* _z_ angular velocity 1.05 rad/s (*)
* _x_ angular acceleration 0 rad/s<sup>2</sup>
* _y_ angular acceleration 0 rad/s<sup>2</sup>
* _z_ angular acceleration 0.5 rad/s<sup>2</sup> 

(*) The values above were derived from giving max RC stick input, but since the platform was originally designed for aerial manipulation, the stick inputs are very slow. Since it was difficult to satisfy the test requirements using our existing autonomy stack, we decided to go with this measurement even though it is very pessimistic.
(**) The OMAV does not roll, and pitching the body was disabled for our customized SubT version due to odometry drift.

The constraints can be found in the following location within the simulation model controller plugin: 
Lines 47-52 in the file src/subt/submitted_models/cerberus_kolibri_sensor_config_1/voliro_controller_plugin/kolibri_controller.hh. 

### Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation
Guide], this vehicle has the following endurance characteristics.

* Battery life of 650 seconds (with 12Ah battery as specified in the Platform Qualification). We plan to test endurance with larger battery payloads.

### Diversions from Physical Hardware of Aerial Scout
* A camera model is used but of course camera sensors are hard to model exactly. The field of view and the resolution are the models matched.
* The controller in the simulation model is different from the one in the real platform. The real controller is a complex third-party IP which we do not have access to. Since this is a unique OMAV, replicating that controller was not deemed feasible. Additionally, since it is an omnidirectional MAV that does not need to roll or pitch, it is well approximated by a simple rigid body under the identified acceleration constraints. On the time scales of navigation, its unmodeled actuator dynamics are negliable.


## <a name="validation_links"></a>Kolibri Validation and Specification Links
* Original OMAV manufucturer: [Link](http://voliro.com)
* LIDAR - Ouster OS0: [Link](https://www.dataspeedinc.com/app/uploads/2020/05/Ouster-OS0-Lidar-Datasheet.pdf)
* Six Grayscale and Color Cameras - Sony IMX287: [Link](https://www.sony-semicon.co.jp/products/common/pdf/IMX273_287_296_297_Flyer.pdf)
* IMU - Bosch BMI085 IMU: [Link](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf)

* Linear and Angular Acceleration and Speed Validation Video: [Link](https://drive.google.com/file/d/181paS8Xs5BYX2FdfhRD7w-OfjpCqRHmQ/view?usp=sharing) 
* Linear and Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/file/d/1vFJhYtik9gus2S0WheV2vLFC43g0EBC4/view?usp=sharing)
* Battery Endurance Validation Video: [Link](https://drive.google.com/file/d/1xXlvUX7FYis6Ne-7e-RfSsRtH_Abe9jK/view?usp=sharing)
* Battery Endurance Validation Data: [Link](https://drive.google.com/file/d/1xXlvUX7FYis6Ne-7e-RfSsRtH_Abe9jK/view?usp=sharing)
