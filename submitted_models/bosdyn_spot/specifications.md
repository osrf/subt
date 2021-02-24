<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# Boston Dynamics Spot
This specifications.md file is a description and proof of virtual model validation for the Boston Dynamics Spot robot base. This robot may be launched using an ign launch command with the variable name `BOSDYN_SPOT`.

## Description
The base body of Boston Dynamics Spot robot with 5 grayscale depth cameras and an IMU.

## Usage Instructions
The robot motion is controlled via standard `cmd_vel` commands.

The joints can also be controlled separately. They offer a force/effort control interface.

There is either one topic for each joint (named `cmd_force/front_left_hip_x` etc.), or a multi-commanding topic `joint_commands`.
This multi-command topic accepts messages of type `sensor_msgs/JointState`. It expects that the `name` and `effort`
fields will be filled with the desired joint names and effort commanded to them, while fields `velocity` and `position`
should contain NaNs (the number of elements in each of these 4 fields should be the same, otherwise the ROS-Ign bridge
will segfault).

## Usage Rights
No additional restrictions have to be taken into account for this model.

### Cost and Scale
* Base vehicle: $60,000

Its weight is approximately 30 kg (TODO). 

### Sensors
The bare body of Spot includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

* 5 grayscale stereo depth cameras (TODO exact type)
* IMU (not exposed via original bosdyn API, but it has to have some)

### Control


### Motion characteristics

Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://www.subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics. 

Maximum forward speed is `1.6 m/s` with an almost instant acceleration to this speed from zero (`TODO m/s^2` from IMU, `TODO m/s^2` from odometry).

Maximum turning speed is `TODO 1 rad/s` with an almost instant acceleration to this speed from zero (`TODO rad/s^2` from both IMU and odometry).

Minimum turning radius is 0, as the robot can turn in place.

### Endurance Characteristics
This configuration has an endurance of approximately 90 minutes.

### Diversions from Physical Hardware of Absolem robot

## Validation and Specification Links
* Vehicle Links:
  * https://www.bostondynamics.com/spot

* Sensor specification links:
  * TODO
    
* Validation Video Links:
  * Endurance test: TODO
  * Maximum linear speed: TODO
  * Maximum angular speed: TODO
  * Rotation in place: TODO
  
* Validation Data Links:
  * Recordings of validation tests: TODO
  * Total mass measurement: TODO

