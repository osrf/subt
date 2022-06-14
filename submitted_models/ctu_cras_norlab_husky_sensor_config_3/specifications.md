<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# CTU-CRAS-NORLAB Husky Sensor Config 3
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-NORLAB Husky with Sensor Configuration 3. 
This robot may be launched using an ign launch command with the variable name `ctu_cras_norlab_husky_sensor_config_3`.

## Description
The main chassis of the robot is the Clearpath Husky robot. 
The sensor rack build by CTU-CRAS provides 4 wide-angle cameras (front, right, rear, left), a 128-beam Ouster lidar and an IMU. 
The robot illuminates the surroundings with LED strips attached to the payload attachment frame on top of the Husky.

Sensor config 3 and 4 are very different from 1 and 2. Configs 3 and 4 represent the husky built by CTU-CRAS for the SubT challenge (which did attend the Finals), while sensor configs 1 and 2 are the build of Norlab, which was only an active backup in the Finals and was never used in the challenge.

The model of the Husky platform is based on the Team MARBLE's Husky, namely on the mesh and texture files; and on the relevant sdf description.
Thanks to the original authors Brett A. Fotheringham(fotheringhambrett@gmail.com), Derek Vasquez (dvasquez@ssci.com), 
Neil Johnson (njohnson@ssci.com) and Hector Escobar (hescobar@ssci.com) for making our lives easier!

## Usage Instructions
The robot motion is controlled via standard `cmd_vel` commands and produces standard odometry.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The Husky has the following estimated commercial costs:
* Base vehicle: $25,000
* Sensor Suite: $14,000 ($2,500 for cameras, $10,000 for the lidar, $1,500 for the IMU) 
* Compute/support electronics and integration: $3,000
* Total: ~ $42,000

Its weight is approximately 58 kg (50 kg the Husky chassis, 8 kg the sensor rig). Two persons are needed for manipulation.

### Sensors
This Husky with sensor configuration 3 includes the following sensors. The specifications are provided below in the [Validation Links](#validation_links) section.

* Ouster OS0-128 3D lidar modeled by `gpu_lidar` sensor. It runs configured to produce 1024x128 scans at 10 Hz. It has a very wide vertical field of view (90 degrees) and short range (about 50 meters).
* 5x RGB global shutter camera Basler a2A1920-51gcPRO with 4 mm lens modeled by `camera` sensor. Each camera provides 86 degrees of horizontal FOV. Resolution is 1920x1200 px. According to datasheet, the camera can run on frequencies up to 51 Hz, but the robot doesn't have sufficient computational and communication capacity to transmit such high frequency data. We thus lowered the framerate of the simulated camera to 9 Hz. We verified in real life that we are able to transmit the 9 Hz images and process them (the 4 cameras more or less saturate a Gigabit link with this framerate).
* XSens MTI-30 IMU: modeled by `imu` sensor.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 4 (real robot will carry 4 or 8).

### Control
The robot is controlled by the DiffDrive plugin. It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  

### Motion Characteristics
The characteristics are identical to the MARBLE Husky robot, which is in turn based on the COSTAR Husky:
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_ velocity range from -1.0 m/s to 1.0 m/s
* _x_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model package:

* `spawner.rb`, line 3

### Endurance Characteristics
We leave the same endurance as the Team MARBLE, i.e. 1 hour. 
Based on the experience from our SubT runs, this value is a fair approximation or even underestimation (depending the type of battery used).

### Diversions from Physical Hardware of MARBLE Husky
Cameras have no distortion as the simulator does not support it.

The 3D lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

## Changing parameters of the robot

Most parameters affecting the simulation model performance are in the `config/` folder, in files `common.yaml`, `sim.yaml` and `ign.yaml`. If some parameter is not exposed to these config files, look into the Xacro files in `urdf/` folder and change the values there.

After changing a value in these files, run script `scripts/update_robot_sdf_ign` to write the changes to the SDF file. Or run `scripts/update_robot_sdf_ign_all_configs` to write the change in SDF files of sensor configs 3 and 4 of Husky.

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__


## Validation and Specification Links
* Most validation data are provided in `marble_husky_sensor_config_1` as this model is based on it.

* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * Ouster OS0-128 https://ouster.com/products/os0-lidar-sensor/
  * Basler cam https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace2/a2a1920-51gcpro/ + lens https://www.baslerweb.com/en/products/vision-components/lenses/basler-lens-c125-0418-5m-p-f4mm/ + Experimental validation: https://drive.google.com/drive/folders/1-KWMvjWSYYeLqPA0C3FCdKuHQAmOHN-z?usp=sharing
  * IMU: XSens MTi-30 https://www.mouser.com/datasheet/2/693/mti-series-1358510.pdf
  * Lights: the robot uses `1.5 m` of LED strips around the body. The total power output of these strips is about `50 W`.
