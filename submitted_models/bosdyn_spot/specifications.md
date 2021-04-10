# Boston Dynamics Spot
This specifications.md file is a description and proof of virtual model validation for the
Boston Dynamics Spot robot base. This robot may be launched using an ign launch command with the
variable name `BOSDYN_SPOT`.

For local testing, you can use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/bosdyn_spot/launch/example.ign robotName:=X1 ros:=true champ:=true teleop:=true

This package contains just the bare robot body which is not expected to be run "standalone" as a robot
in the SubT competition. It is expected that each team will develop their own sensor config based
on this common model. So far, it is known that teams CTU-CRAS-Norlab, MARBLE and CoSTAR use the Spot
robot and a sensor config from these teams can be expected. But it is not limited to these teams.

Although it is not expected to run this robot "standalone", it works. But there might be some scripts
that expect the `_sensor_config_x` suffix. We have left out this suffix intentionally in this base
package so that it is not common that the bare robot is used.

The additional sensor configs reuse a lot of files from this base package, including the meshes.
For that to work when the model is downloaded from the Fuel database, each sensor config has to
specify `BOSDYN_SPOT` as a dependency in their `model.config` file.

As an example of how to "extend" this base robot with a custom sensor config, see
[ctu_cras_norlab_spot_sensor_config_1](../ctu_cras_norlab_spot_sensor_config_1).

## Description
The base body of Boston Dynamics Spot robot with 5 mono fisheye cameras, 5 depth cameras and an IMU.
The robot is a quadruped with many autonomous motion functions which are mostly not modeled in this
simulation package.

## Usage Instructions

### Supporting package for cmd_vel support

The robot model as it is in the SubT repo can only be controlled joint-by-joint (see further).
To get `cmd_vel` command support via a basic open-loop gait generator, the users can download
the support package [ctu-vras/subt_spot](https://github.com/ctu-vras/subt_spot) to their solution
container. It is also possible to use any other gait generation package.

With the support package, robot motion is controlled via standard `cmd_vel` commands. Pose of the
body relative to ground can be controlled by `geometry_msgs/Pose` commands on topic `body_pose`.
See the usage instructions in the package's README.md file for further instructions on how to run it.

See the [Control section](#control) for more options how to control the robot.

### Contacts from simulation

The simulator publishes information about contact of the lower legs with terrain. There are two
types of topics. Both depend on the presence of plugin `ignition-gazebo-contact-system` in the
simulation world. This plugin is automatically a part of all competition worlds, but if you create
your own worlds, you may need to add it manually.

First type of contact information is the per-leg precise set of contacts. It is published on topic
`leg_contacts/front_left` etc. of type `bosdyn_spot/ContactsStamped` and contains the exact positions
of all contact points reported by the simulator.

Second type of contact information is published on topic `leg_contacts` of type `bosdyn_spot/LogicalContact`.
This message contains just a true/false value specifying which legs were in contact at all. The order of
the legs is `front_left`, `front_right`, `rear_left` and `rear_right`. If you run the support package,
this topic is further retranslated to topic `foot_contacts` of type `champ_msgs/ContactsStamped` which is
used in state estimation.

### Odometry

If you run the support package, there is also a basic leg odometry for the robot. It is not very precise,
but can be used to get at least a glimpse into how does the robot move.

There are two parts the odometry consists of: base-footprint part and footprint-odom part.

The base-footprint odometry is published on topic `base_to_footprint_pose` of type
`geometry_msgs/PoseWithCovarianceStamped` and defines the pose of the body `base_link` frame
relative to the projection of the robot to the (flat) ground. This odometry enters a Kalman filter
which produces TF frame `base_link->base_footprint` (both prefixed by the robot name). A sample
configuration of this filter is in `config/ekf/base_to_footprint.yaml`, but the users can use their
own configuration.

The footprint-odom part is published on topic `odom/raw` of type `nav_msgs/Odometry`. This is the
transformation between the robot's footprint and the odom frame. This odometry enters a Kalman filter
which produces TF frame `base_footprint->odom` (both prefixed by the robot name). A sample
configuration of this filter is in `config/ekf/footprint_to_odom.yaml`, but the users can use their
own configuration.

## Usage Rights
No additional restrictions have to be taken into account for this model.

It is based on the BSD-licensed packages [spot_ros](https://github.com/clearpathrobotics/spot_ros)
and CHAMP's [spot_description](https://github.com/chvmp/spot_ros/tree/gazebo/spot_description).

## Cost and Scale
* Base vehicle: $75,000

Its weight is approximately 34 kg. 

## Sensors
The bare body of Spot includes the following sensors. The specifications for these instruments are not
public, so their properties and performance was estimated from real robot data.

* 5 mono fisheye cameras with resolution 640x480 px modeled by `camera` sensor and `MonoCameraSystem` plugin
  that applies a simple transformation to the original RGB images the simulator produces.
* 5 stereo cameras with resolution 424x240 px modeled by `depth_camera` sensor.
* IMU modeled by the `imu` sensor (not exposed via original bosdyn API, but it has to have some).
* Foot contact sensors publishing the contact data to topic `leg_contacts`.

Additional sensor configs of Spot are expected to add a lot of payload and thus a separate `specifications.md`
file is expected to be a part of the sensor config packages.

## Control

Even without the support package, the joints can be controlled separately.
They offer both a positional and a force/effort control interface.

There are several control topics that repeat for every joint and contain the joint name in their name.
The robot has 4 legs: `front_left`, `front_right`, `rear_left` and `rear_right`. Each leg has 3 joints:
`hip_x`, `hip_y` and `knee`. So there are 12 joints in total, and their name follows pattern `$leg_$joint`.
All of these joints are revolute with limits.

### Single-joint control

#### Effort control

To control each joint separately via commanded effort, there are topics `cmd_force/front_left_hip_x` etc.
of type `std_msgs/Float64`. There might be time sync problems when commanding each joint separately, so
it is suggested to not used this interface and use the multi-commanding topic described further.

#### Position control

To control each joint separately via target position, there are topics `cmd_pos/front_left_hip_x` etc.
of type `std_msgs/Float64`. There might be time sync problems when commanding each joint separately, so
it is suggested to not used this interface and use the multi-commanding topic described further. Each
joint contains a PID controller that translates the positional commands into commanded effort (but for
performance reasons, it does not republish the commanded effort to the `cmd_force/*` topics - it
communicates directly with the simulator instead). The PID gains of all positional controllers are
configurable during runtime via standard ROS dynamic reconfigure. You can either reconfigure each PID
separately (e.g. `joint_group_position_controller/gains/front_left_hip_x`) or you can set all 12
PIDs at once using namespace: `pid_multituner`.

### Multi-joint control

There are two interfaces that allow commanding a set of joints at the same time (synchronously in a
single simulation step). The is the preferred option because it should make the control algorithms
more stable. Both interfaces are equivalent in what they can do and they differ just in the type of
the input message.

First interface is `joint_commands` of type `sensor_msgs/JointState`. It expects that the `name`
field will contain a list of joints that should be affected by this command. In addition to this field,
it is expected to contain finite values in either `effort` or `position`. The order of these numbers
corresponds to the order of joints in the `name` field. Both `effort` and `position` can be defined
at the same time, which means the effort from PID will add with the commanded `effort`, but it probably
doesn't make much sense. All the remaining fields (`velocity` and possibly `effort` or `position`) have
to have the same length as `name` and be filled with `NaN`s. This means that these values will not be
set. If you forget to make all the 4 arrays in the message of the same length, the `ros_ign` bridge for
`joint_commands` topic might segfault.

The alternative multi-joint control topic is `joint_group_position_controller/command` accepting
`trajectory_msgs/JointTrajectory` messages. Their treatment is similar to `joint_commands`, but the
structure of the message is slightly different. The command type supports passing a trajectory with
multiple points, but this package only executes the first point in the trajectory (and it ignores the
`time_from_start` field and executes the command immediately). The trajectory command does not need
to have all fields of the same length. You can leave the unused fields empty.

### Turning off positional control

Once a joint has been commanded a positional control command, it tries to keep this position.
To turn off this behavior (e.g. to allow for usage of force-control algorithms), send a `NaN`
command to each separate joint. This will not work with the multi-joint command topics as they
ignore all `NaN` values and do not pass them further to the joints.

When the robot is spawned into simulation, each joint is commanded a positional control command
that specifies that the joint should keep its initial position.

## Motion characteristics

Based on the tests specified in the DARPA SubT Challenge 
[Model Preparation Guide](https://www.subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf),
this vehicle has the following motion constraint characteristics.

The tests have not yet been conducted due to Covid restrictions and unavailability of the real hardware.
Team CTU-CRAS-Norlab is planning to rent a Spot at the end of April or beginning of May, so that is
when we will fill the missing validation data. But we hope that team CoSTAR or MARBLE will be able
to do the tests earlier.

Maximum forward speed is `1.6 m/s` with an almost instant acceleration to this speed from zero
(`TODO m/s^2` from IMU, `TODO m/s^2` from odometry).
Configuration of the maximum forward speed in simulation has no single defining value. It is a
result of the configuration of gait and and PID controllers. There is a default velocity smoother
in the support package that caps the _commanded_ velocity to 1.5 m/s.

Maximum turning speed is `TODO 1 rad/s` with an almost instant acceleration to this speed from zero
(`TODO rad/s^2` from both IMU and odometry).

Minimum turning radius is 0, as the robot can turn in place.

The real robot can climb stairs. The simulated model should be able to do it too, provided a suitable
control algorithm.

## Endurance Characteristics
This configuration has an endurance of approximately 90 minutes (value from datasheet).

## Diversions from Physical Hardware of Spot robot
The fisheye lens does not use any fisheye projection as the simulator doesn't support that.

Cameras have no distortion as the simulator doesn't support it.

The simulated model offers much inferior autonomous motion capabilities than the real platform.

## Changing parameters of the robot
Most parameters affecting the simulation model performance are in the `config/model` folder,
in files `common.yaml` and `ign.yaml`. If some parameter is not exposed to these config files,
look into the Xacro files in `urdf/` folder and change the values there.

After changing a value in these files, run script `scripts/update_robot_sdf` to write the
changes to the `model.sdf` file. This updates the SDF just for this particular robot. If the change
is supposed to propagate in all dependent sensor configs, `update_robot_sdf` of each of the
respective configs must be called. After the update scripts are called, review the resulting SDF and
commit the changes.

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

The URDF files are not physically present, but are generated on-the-fly by calling script
`scripts/print_robot_urdf`.

## Validation and Specification Links
* Vehicle Links:
  * https://www.bostondynamics.com/spot

* Sensor specification links:
  * N/A (no specification of the used sensors has been published)
    
* Validation Video Links:
  * Endurance test: TODO
  * Maximum linear speed: TODO
  * Maximum angular speed: TODO
  * Rotation in place: TODO
  
* Validation Data Links:
  * Recordings of validation tests: TODO
  * Total mass measurement: TODO

