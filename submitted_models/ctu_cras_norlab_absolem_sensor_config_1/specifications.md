<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# CTU-CRAS-Norlab Absolem Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-Norlab Absolem robot with Sensor Configuration 1. This robot may be launched using an ign launch command with the variable name `CTU_CRAS_NORLAB_ABSOLEM_SENSOR_CONFIG_1`.

## Description
This configuration is based BlueBotics Absolem tracked robot. The robot is equipped with 4 additional subtracks (flippers) and a rotating 2D lidar.

## Usage Instructions
The robot motion is controlled via standard `cmd_vel` commands.

Flippers can be controlled by publishing to topics `flippers_cmd_vel/front_left` (`front_right`, `rear_left`, `rear_right`) (`std_msgs/Float64`) for velocity control or topics `flippers_cmd_pos/front_left`, etc. for position control. Relative positional control is available on `flippers_cmd_pos_rel/front_left` etc. Maximum angular velocity of the flippers is `pi/4 rad/s`. The effort limits in the model were set so that the robot can support itself with the flippers, but cannot use them to lift itself on all four flippers. This is how the real flippers work. The current position of the flippers is published to `joint_states` as `front_left_flipper_j` etc. The flippers can continuously rotate.

The laser rotation is velocity-controlled by publishing to topic `lidar_gimbal/roll_rate_cmd_double` (`std_msgs/Float64`). The laser has hard stops at `+-2.36 rad` and maximum rotation velocity is `1.2 rad/s`. The laser has an automatic controller that reverses the rotation direction at a given angle (currently ca. `1.6 rad`). The current position of the laser is published to `joint_states` as `laser_j`. The default (zero) position of the laser is such that the scanning plane is levelled with ground.

The robot is equipped with two more passive joints which connect the tracks to `base_link`. They are called `left_track_j` and `right_track_j`. These joints are connected via a differential with lockable brake. The differential makes sure that `angle(left_track_j) == -angle(right_track_j)` at all times. This model configuration has the differential brake applied in zero position, which means the tracks cannot move relative to the robot body. 

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The Absolem robot has the following estimated costs:
* Base vehicle: $12,000 (very coarse estimate, only 5 pieces were manufactured)
* Sensor Suite: $8,100 ($7,000 for Ladybug LB-3, $1,000 for Sick LMS-151, $100 for Realsense D435).
* Compute/support electronics: $2,500
* Total: ~ $22,600

Its weight is approximately 38 kg (possible, but heavy to heft with a single person). 

### Sensors
This Absolem with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Pointgrey Ladybug LB-3 omnidirectional camera: modeled by 6 `camera` plugins. The physical "multicamera" also has six distinct cameras, and the simulated ones closely follow their placement. Each camera has resolution `1616x1232 px` with a field of view of `78 deg`. The camera captures 6 synchronized "multiimages" per second.
* D435 RGBD Camera, modeled by `rgbd_camera` plugin
  - 1x fixed, downward-facing at about 30 degrees (for examining terrain)
* Sick LMS-151 2D lidar, modeled by `gpu_lidar` plugin
* XSens MTI-G 710 IMU: modeled by `imu_sensor` plugin.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2.

### Control
This robot is controlled by the DiffDrive plugin.  It accepts twist inputs which drives the vehicle along the x-direction and around the z-axis. We add additional 8 pseudo-wheels where the robot's tracks are to better approximate a track vehicle (flippers are subdivided to 5 pseudo-wheels). Currently, we are not aware of a track-vehicle plugin for ignition-gazebo.  A TrackedVehicle plugin does exist in gazebo8+, but it is not straightforward to port to ignition-gazebo.  We hope to work with other SubT teams and possibly experts among the ignition-gazebo developers to address this in the future.

Flippers provide interfaces for velocity control and absolute and relative positional control. The positional controllers move flippers to the given position using the maximum speed of the flippers.

### Motion characteristics

Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://www.subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics. 

Maximum forward speed is `0.4 m/s` with an almost instant acceleration to this speed from zero (`3 m/s^2` from IMU, `1.4 m/s^2` from odometry). The physical robot can go even a little faster, but then it generates so much vibration that all sensor data are practically useless. We chose this speed as a reasonable limit.

Maximum turning speed is `0.5 rad/s` with an almost instant acceleration to this speed from zero (`4 rad/s^2` from both IMU and odometry).

There is also a maximum forward speed of each track, which is `0.4 m/s`. This means that in case forward and rotation speeds are combined in a single command, the individual speed limits might not be reached because of the track speed limitation.

Minimum turning radius is 0, as the robot can turn in place.

Flippers can rotate around their joint at a rate of approximately `0.8 rad/s`.

The laser can rotate at a rate of `1.2 rad/s` within `-2.38 .. 2.38 rad` bounds.

### Endurance Characteristics
This configuration has an endurance of approximately 2 hours. The endurance test showed a run time of 2 hours and 6 minutes until fully charged lithium-ion batteries ran under 3.3 V/cell. The robot could run even a little longer, but we did not want to put strain on the batteries.

### Diversions from Physical Hardware of Absolem robot
There is a little "tower" (or a "rod") to which we attach our communication device (Mobilicom MCU-30 Lite). This device is modeled just by the generic SubT comms plugin.

The tracks and flippers have to be approximated by wheels, as DartSim/Ignition Gazebo have no support for tracked vehicles. There is a working model for ODE/Gazebo, but there is no straight way of transferring it to Ignition Gazebo. This approximation results in worse performance on obstacles, and it can even happen that a piece of terrain gets "stuck" right between two wheels and the robot would completely stop in a case that would not be a problem with real tracks. 

## Validation and Specification Links
* Vehicle Links:
  * https://www.bluebotics.com/media/Brochure-Mobile-Robotics.pdf?462722

* Sensor specification links:
  * https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms151-10100/p/p141840
    - The minimum range from datasheet is `0.5 m`, but we found out that data starting from `0.1 m` can be used if a systematic error is compensated.
  * D435 RGBD Camera - https://www.intelrealsense.com/depth-camera-d435/
  * PointGrey Ladybug LB-3 omnicamera - https://flir.app.boxcn.net/s/ds1bkoq9eiq6ga714nmnmzgpotgd4gkf/file/418658016565
  * IMU: XSens MTi-G 710 - https://www.xsens.com/hubfs/Downloads/Leaflets/mti-g-710-series.pdf
  * Lights: the robot uses `1 m` of LED strips around the body. The total power output of these strips is about `20 W`.
    
* Validation Video Links:
  * Endurance test: Our camera ended each shot after 28 minutes, and there was also a depleted camera battery for about 20-30 minutes until the personnel noticed the problem. Thus we had to upload the video in parts: https://www.youtube.com/watch?v=h-Do-KO95zQ, https://www.youtube.com/watch?v=s8UMUY6W91o, https://www.youtube.com/watch?v=Gd6QGNa3TIY,  https://www.youtube.com/watch?v=zLrsgmCoFgU, https://www.youtube.com/watch?v=L2g-bApQsSE
  * Maximum linear speed: https://youtu.be/9v2lv2tw-xM
  * Maximum angular speed: https://youtu.be/CKjDstWgI9M
  * Rotation in place: https://youtu.be/CKjDstWgI9M
  * Flipper motion: https://youtu.be/OCGHh0HdC_4
  * Differential motion: https://youtu.be/eyesmU_BmUI

* Validation Data Links:
  * Recordings of validation tests: https://login.rci.cvut.cz/data/darpa-subt/data/bagfiles/virtual_qual_absolem/
  * Total mass measurement: https://login.rci.cvut.cz/data/darpa-subt/data/share/virtual_qual_absolem/

