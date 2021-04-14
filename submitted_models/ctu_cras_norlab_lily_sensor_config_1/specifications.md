# CTU-CRAS-NORLAB Lily hexapod robot
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-NORLAB Lily robot.

## Description
Lily is a six-legged robot with great static stability and ability to overcome rough terrains. The robot consists of a body with computational units and sensors and six legs, each with three degrees of freedom (DoF) actuated by HEBI servomotors. The real robot is also fully waterproof. The base platform is the [HEBI robotics Lily hexapod robot kit](https://www.hebirobotics.com/robotic-kits).  
For the purpose of the DARPA Subterranean challange the considered sensory payload of the Lily hexapod is:
* LIDAR
* Custom omnicamera bundle
* IMU
* Gas sensor
* Optionally 6 communication breadcrumbs (in sensor config 2)

This ignition gazebo model is based on the official gazebo model of the [HEBI Lily kit](https://github.com/HebiRobotics/hebi_description/tree/cwbollinger/daisy_urdf). The parameters of simulated servomotors are set according to the real servomotors parameters including joint effort limits. 

## Dependencies
To be able to control this robot model using velocity commands you need an additional package. Please clone the repository [ctu_cras_norlab_hexapod_controller](https://github.com/comrob/ctu_cras_norlab_hexapod_controller) and follow the instructions specified in its README.

## Usage Instructions 
This robot may be launched using an ign launch command with the variable name `CTU_CRAS_NORLAB_LILY_SENSOR_CONFIG_1`.

For local testing, you can use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/ctu_cras_norlab_lily_sensor_config_1/launch/example.ign robotName:=X1

## Usage Rights
The software included in this package is released under a [Apache License 2.0](LICENSE) license.

Lily model is adapted from [hebi_description/](https://github.com/HebiRobotics/hebi_description.git) repository, released under [Apache License 2.0](LICENSE) license.


## Cost and Scale
* Base vehicle: $75,000
* Custom omnicamera solution: $5,000
* IMU: $1,500
* Computers: $500
* Lidar: $10,000
* Total price: $92,000

Its weight is approximately 32.7 kg.

## Sensors
The following specific sensors are declared payloads of this vehicle.

* Ouster OS0-128 3D lidar modeled by `gpu_lidar` sensor. It runs configured to produce 2048x128 scans at 10 Hz. It has a very wide vertical field of view (90 degrees) and short range (about 50 meters).
* 5x RGB global shutter camera Basler a2A1920-51gcPRO with 4 mm lens modeled by `camera` sensor. Each camera provides 86.5 degrees of horizontal FOV. Resolution is 1920x1200 px. According to datasheet, the camera can run on frequencies up to 51 Hz, but the robot doesn't have sufficient computational and communication capacity to transmit such high frequency data. We thus lowered the framerate of the simulated camera to 9 Hz. We verified in real life that we are able to transmit the 9 Hz images and process them (the 5 cameras more or less saturate a Gigabit link with this framerate).
* XSens MTI-30 AHRS: modeled by `imu` sensor.
* 6 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2 (real robot will carry 4 or 8 breadcrumbs).

## Control
The Lily robot is controlled by custom package `hexapod_controller`, available in the repository [ctu_cras_norlab_hexapod_controller](https://github.com/comrob/ctu_cras_norlab_hexapod_controller).

The controller allows for steering the robot using the `cmd_vel` command via basic open-loop locomotion control.


## Motion characteristics
Based on the tests specified in the DARPA SubT Challenge 
[Model Preparation Guide](https://www.subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf),
this vehicle has the following motion constraint characteristics.

The tests have not yet been conducted due to Covid restrictions and unavailability of the real hardware.
Team CTU-CRAS-Norlab is in the process of purchasing of the platform that should be available during the late May when we will supply the missing validation data.

Maximum forward speed according ot the base platform datasheet is `0.43 m/s` with an almost instant acceleration to this speed from zero
(`TODO m/s^2` from IMU, `TODO m/s^2` from odometry).
Configuration of the maximum forward speed in simulation has no single defining value. It is a
result of the configuration of gait and PID controllers. 

Maximum turning speed is `TODO 1 rad/s` with an almost instant acceleration to this speed from zero
(`TODO rad/s^2` from both IMU and odometry).

Minimum turning radius is 0, as the robot can turn on the spot.

The real robot can climb stairs. The simulated model should be able to do it too, provided a suitable
control algorithm.

## Endurance characteristics
This configuration has an endurance of approximately 75 minutes (value from datasheet discounted by projected consumption of the additional computational units and sensors)

The endurance is limited to 60 minutes until validation data are provided.

## Diversions from Physical Hardware of HEBI Lily robot
Provided data and configuration options of the real HEBI servomotors are surpasing the possibilities of the simulator.
The real servos provide the synchronized Position, Velocity, True torque, Current, Voltage, and 3-DoF acceleration (embedded IMU in each servomotor) with the update rate of 1 kHz.

Cameras have no distortion as the simulator doesn't support it.

The Ouster lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

## Changing parameters of the robot
Parameters affecting the simulation model performance are in the Xacro files in `urdf/` folder. 
After changing a value in these files, run script `scripts/convert.sh` to write the
changes to the `lily.sdf` file. 

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

The URDF files are not physically present, but are generated on-the-fly in desctipion.launch by Xacro.
They can also be created on demand by calling script `scripts/xacro2urdf.sh`.

## Validation and Specification Links
* Vehicle links:
  * Base platform: [HEBI robotics Lily hexapod robot kit](https://www.hebirobotics.com/robotic-kits)

* Sensor specification links:
  * [Ouster OS0-128](https://ouster.com/products/os0-lidar-sensor/)
  * [Basler cam](https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace2/a2a1920-51gcpro/) + [lens](https://www.baslerweb.com/en/products/vision-components/lenses/basler-lens-c125-0418-5m-p-f4mm/) + Experimental validation: https://drive.google.com/drive/folders/1-KWMvjWSYYeLqPA0C3FCdKuHQAmOHN-z?usp=sharing
  * IMU: [XSens MTi-30](https://www.mouser.com/datasheet/2/693/mti-series-1358510.pdf)
  * Lights: the robot uses LED strips around the body. The total power output of these strips is about `30 W`. It also has a ceiling-pointing light.

* Validation Video Links:
  * Endurance test: TODO
  * Maximum linear speed: TODO
  * Maximum angular speed: TODO
  * Rotation in place: TODO
  
* Validation Data Links:
  * Recordings of validation tests: TODO
  * Total mass measurement: TODO

