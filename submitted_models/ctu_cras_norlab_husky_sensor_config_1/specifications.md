<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# CTU-CRAS-NORLAB Husky Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-NORLAB Husky with Sensor Configuration 1. 
This robot may be launched using an ign launch command with the variable name `ctu_cras_norlab_husky_sensor_config_1`.

## Description
The main chassis of the robot is the Clearpath Husky robot. 
The sensor rack build by CTU-CRAS-NORLAB provides 6 wide-angle cameras (5 arranged in a pentagon, one top-oriented), a 16-beam lidar from RoboSense, a rear-oriented Intel Realsense D435i and an IMU. 
The robot illuminates the surroundings with LED spot lights positioned close to the cameras and having similar beam angles to fill the camera FoVs.

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
* Sensor Suite: $10,000 ($5,000 for cameras, $3,000 for the lidar, $100 for Realsense D435, $2000 for the IMU) 
* Compute/support electronics and integration: $5,000
* Total: ~ $40,000

Its weight is approximately 61 kg (50kg the Husky chassis, 11kg the sensor rig). Two persons are needed for manipulation.

### Sensors
This Husky with sensor configuration 1 includes the following sensors. The specifications are provided below in the [Validation Links](#validation_links) section.

* Robosense RS-LiDAR-16 modeled by the `gpu_lidar` plugin
* 6x Basler Ace2 (a2A1920-51gcPRO) camera and C125-0418-5M-P Basler Lens modeled by the `camera` plugin
* D435i RGBD Camera, pointing towards rear and modeled by the `rgbd_camera` plugin
* Xsens MTi-100 IMU, modeled by the `imu_sensor` plugin

### Control
The robot is controlled by the DiffDrive plugin. It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  

### Motion Characteristics
The characteristics are identical to the MARBLE Husky robot, which is in turn based on the COSTAR Husky:
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_ velocity range from -1.0 m/s to 1.0 m/s
* _x_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>

The constraints can be found in the following locations within the simulation model package:

* `spawner.rb`, lines 23-26

### Endurance Characteristics
We leave the same endurance as the Team MARBLE, i.e. 1 hour. 
Based on the experience from our SubT runs, this value is a fair approximation.

### Diversions from Physical Hardware of MARBLE Husky
The sensor rack visualization is based on our CAD file of the robot and misses wiring, computer components located in the robot bay and light bulbs. 
We consider these details irrelevant for the simulation.

Cameras have no distortion as the simulator does not support it.

The 3D lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

## Validation and Specification Links
* Most validation data are provided in `marble_husky_sensor_config_1` as this model is based on it.

* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * Robosense RS-LiDAR-16 - https://www.robosense.ai/en/rslidar/RS-LiDAR-16
  * D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i
  * Basler Ace2 a2A1920-51gcPRO - https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace2/a2a1920-51gcpro
  * Basler Lens C125-0418-5M-P - https://www.baslerweb.com/en/products/vision-components/lenses/basler-lens-c125-0418-5m-p-f4mm
  * Xsens MTi-100 IMU - https://www.xsens.com/products/mti-100-series
