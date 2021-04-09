# CTU-CRAS-Norlab MARV robot
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-Norlab MARV (Mobile Autonomous Rescue Vehicle) robot. This robot may be launched using an ign launch command with the variable name `CTU_CRAS_NORLAB_MARV_SENSOR_CONFIG_1`.

For local testing, you can also use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/ctu_cras_norlab_marv_sensor_config_1/launch/example.ign robotName:=X1 ros:=true teleop:=true

## Description
This configuration is a custom-made tracked robot manufactured by company JettyVision. The robot is equipped with 4 tracks (flippers) and a 3D lidar.

## Usage Instructions
The robot motion is controlled via standard `cmd_vel` commands.

Flippers can be controlled by publishing to topics `flippers_cmd_vel/front_left` (`front_right`, `rear_left`, `rear_right`) (`std_msgs/Float64`) for velocity control or topics `flippers_cmd_pos/front_left`, etc. for position control. Relative positional control is available on `flippers_cmd_pos_rel/front_left` etc. Maximum angular velocity of the flippers is `1 rad/s`. The effort limits in the model were set so that the robot can support itself with the flippers and lift itself on all four flippers. This is how the real flippers work. The current position of the flippers is published to `joint_states` as `front_left_flipper_j` etc. The flippers can continuously rotate.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

## Cost and Scale
MARV robot is result of custom production contracted to JettyVision company as the winner of [tender VZ0086096: USAR robot with manipulation arm](https://tenderarena.cz/dodavatel/seznam-profilu-zadavatelu/detail/Z0003010/zakazka/319126). The cost of the contract is approx. $300,000 including sensors and a robotic arm (except our customized omnicamera solution). We do not use the arm in SubT competition.

* Base vehicle: $250,000 (single custom production piece; price for the whole year-long design process and delivery of prototype; further pieces would be much cheaper)
* Custom omnicamera solution: $5,000
* Total: ~ $255,000

Its weight is approximately 61 kg (better to carry in two people, or one person when the 10 kg batteries are taken out). This is an estimate based on CAD models of the robot and the sensors. Precise measurement will be provided in May 2021.

## Sensors
MARV with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Ouster OS0-128 3D lidar modeled by `gpu_lidar` sensor. It runs configured to produce 2048x128 scans at 10 Hz. It has a very wide vertical field of view (90 degrees) and short range (about 50 meters).
* 2x RGB global shutter camera Basler acA2040-35gc with fisheye lens modeled by `camera` sensor. Each camera provides a horizontal FOV of 138 degrees. Resolution of the camera is 2048x1536 px. The datasheet framerate is up to 36 Hz, but we do not have enough computational/communication capacity to process the images. So the frame rate in simulation is decreased to 15 Hz.
* 5x RGB global shutter camera Basler a2A1920-51gcPRO with 4 mm lens modeled by `camera` sensor. Each camera provides 86 degrees of horizontal FOV. Resolution is 1920x1200 px. According to datasheet, the camera can run on frequencies up to 51 Hz, but the robot doesn't have sufficient computational and communication capacity to transmit such high frequency data. We thus lowered the framerate of the simulated camera to 9 Hz. We verified in real life that we are able to transmit the 9 Hz images and process them (the 5 cameras more or less saturate a Gigabit link with this framerate).
* 4x TFMini plus point lidar modeled by `gpu_lidar` sensor with 9 rays covering a field of view of 3.6 degrees, which is the light ray spread specified in datasheet. The point lidar works from 0.1 to 12 meters and has distal resolution of 1 cm.
* XSens MTI-30 IMU: modeled by `imu` sensor.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configurations 2 an 4 (real robot will carry 4 or 8).
* Sensor configurations 3 and 4 carry an additional thermocamera FLIR Boson with 92 degrees horizontal FOV lens, modeled by `thermal` sensor. Resolution of the camera is 320x256 px at 8.6 Hz, and temperature range is approximately -50 °C up to 180 °C.
* Custom-made flood sensors alerting the robot that it entered a flooded area (not modeled).

## Control
This robot is controlled by the DiffDrive plugin.  It accepts twist inputs which drives the vehicle along the x-direction and around the z-axis. We add additional 8 pseudo-wheels where the robot's flippers are to better approximate a tracked vehicle. Currently, we are not aware of a track-vehicle plugin for ignition-gazebo.  A TrackedVehicle plugin does exist in gazebo8+, but it is not straightforward to port to ignition-gazebo.  We hope to work with other SubT teams and possibly experts among the ignition-gazebo developers to address this in the future.

Flippers provide interfaces for velocity control and absolute and relative positional control. The positional controllers move flippers to the given position using the maximum speed of the flippers.

## Motion characteristics

The robot is still in development phase so we had to extract the following values from the available CAD drawings and other design documents. We plan to back the theoretical values with real-world measurements as soon as the robot is finished (May 2021). We intentionally choose more conservative values to not overshoot the real robot capabilities. 

Maximum forward speed is `1.0 m/s` with an almost instant acceleration to this speed from zero (set from [config/common.yaml#L30]()).

Maximum turning speed is `1.0 rad/s` with an almost instant acceleration to this speed from zero.

There is also a maximum forward speed of each track, which is `1.0 m/s`. This means that in case forward and rotation speeds are combined in a single command, the individual speed limits might not be reached because of the track speed limitation.

Minimum turning radius is 0, as the robot can turn in place.

Flippers can rotate around their joint at a rate of approximately `1.0 rad/s` (set from [config/common.yaml#L32]()).

The strength of the flippers is set so that the robot can lift itself on the flippers and drive even in the lifted state. The effort required for this behavior is set in [config/common.yaml#L33]().

## Endurance Characteristics
MARV uses two custom-produced battery packs that are suitable for transport by airplanes. Each battery pack is a module consisting of 8 99.9 Wh lithium-ion batteries for drones. The total capacity of the battery pack is thus 1.6 kWh. Onboard computers and sensors should not take more than 150 W, so there is plenty of capacity left to the motors. Extrapolating our real-life experience with the Absolem robot, MARV should definitely be able to drive and run all algorithms for 90 minutes (probably much more).

We will update this section with real-world measured values as soon as the robot is ready for testing (May 2021).

The vehicle's endurance is currently set to 60 minutes until validation data can be provided.

## Diversions from Physical Hardware of MARV robot
The tracks and flippers have to be approximated by wheels, as DartSim/Ignition Gazebo have no support for tracked vehicles. There is a working model for ODE/Gazebo, but there is no straight way of transferring it to Ignition Gazebo. This approximation results in worse performance on obstacles, and it can even happen that a piece of terrain gets "stuck" right between two wheels and the robot would completely stop in a case that would not be a problem with real tracks.

The point lidars are not modeled as a single ray, but rather as an array of 3x3 rays, to simulate the cone the used light source produces. The values should be averaged.

The fisheye lens does not use any fisheye projection as the simulator doesn't support that.

Cameras have no distortion as the simulator doesn't support it.

The Ouster lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

The flippers do not collide with the other ones. The SDF is prepared for that and it is sufficient to turn on self_collide for the whole model, but we were unable to make whole-body rotation plausible with this flag active. Without self-collision, rotation of the robot body is OK. Friction cone model would probably help with that, but it is still unclear why self_collide affects the rotation.

## Changing parameters of the robot

Most parameters affecting the simulation model performance are in the `config/` folder, in files `common.yaml`, `sim.yaml` and `ign.yaml`. If some parameter is not exposed to these config files, look into the Xacro files in `urdf/` folder and change the values there.

After changing a value in these files, run script `scripts/update_robot_sdf_ign` to write the changes to the SDF file. Or run `scripts/update_robot_sdf_ign_all_configs` to write the change in SDF files of all sensor configs of MARV.

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

## Validation and Specification Links
* Vehicle Links:
  * https://tenderarena.cz/dodavatel/seznam-profilu-zadavatelu/detail/Z0003010/zakazka/319126
* Sensor specification links:
  * Ouster OS0-128 https://ouster.com/products/os0-lidar-sensor/
  * Basler cam https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace/aca2040-35gc/ + fisheye lens https://www.mouser.com/datasheet/2/857/DG00212701000_Tech_Spec_for_SAP_2000036382_2000036-1628255.pdf
  * Basler cam https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace2/a2a1920-51gcpro/ + lens https://www.baslerweb.com/en/products/vision-components/lenses/basler-lens-c125-0418-5m-p-f4mm/ + Experimental validation: https://drive.google.com/drive/folders/1-KWMvjWSYYeLqPA0C3FCdKuHQAmOHN-z?usp=sharing
  * Benewake TFmini plus point lidar https://www.mouser.com/datasheet/2/1099/Benewake_10152020_TFmini_Plus-1954028.pdf
  * IMU: XSens MTi-30 https://www.mouser.com/datasheet/2/693/mti-series-1358510.pdf
  * Boson thermal camera: https://www.oemcameras.com/flir-boson-320x256-2mm.htm
  * Lights: the robot uses `1.5 m` of LED strips around the body. The total power output of these strips is about `50 W`. It also has a ceiling-pointing spot light.

* Validation Video Links:
  * Endurance test: TBD (May 2021)
  * Maximum linear speed: TBD (May 2021)
  * Maximum angular speed: TBD (May 2021)
  * Rotation in place: TBD (May 2021)
  * Flipper motion: TBD (May 2021)

* Validation Data Links:
  * Recordings of validation tests: TBD (May 2021)
  * Total mass measurement: TBD (May 2021)
