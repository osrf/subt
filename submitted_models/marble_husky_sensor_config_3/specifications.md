<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE Husky Sensor Config 3
This specifications.md file is a description and proof of virtual model validation for the MARBLE Husky with Sensor Configuration 3. This robot may be launched using an ign launch command with the variable name `marble_husky_sensor_config_3`.

## Description
This configuration is based on Clearpath Robotics Husky ground robot. The marble sensor suite is located at the front of the husky and includes stationary sensors and some sensors and two lights for forward and backward facing cameras.  

## Usage Instructions
The robot can be used in the same manner as the COSTAR Husky robot.  It accepts twist messages on the cmd_vel topic. 
## Usage Rights
The same Rights are granted for the configuration as for the COSTAR Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE Husky has the following estimate commercial cost components:
* Base vehicle: $25,000
* Sensor suite: $14,000
* Compute/Support Electronics: $4,000
* Total: ~ $43,000

Its weight is approximately 85 lbs (two-person heft).

### Sensors
This MARBLE Husky with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* D435i RGBD Camera, modeled by rgbd_camera plugin
  - fixed, downward-facing at about 45 degrees (for examining terrain)
* Ouster 3D Lidar (64 Channel), modeled by gpu_lidar plugin
* Microstrain IMU: 3DM-GX5-25, modeled by imu_sensor plugin. Notes on modeling of the IMU are included in the model.sdf file.  (located under the 3D lidar, installed at same x,y location as 3D lidar)
* RPLidar S1 Planar Lidar (under the 3D lidar), modeled by gpu_ray plugin
* HD MIPI Cameras - 720p (x2)
  - (1x) forward facing on top of sensor tower
  - (1x) rear facing on top of sensor tower

### Control
This MARBLE Husky is controlled by the DiffDrive plugin.  It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics.

The motion characteristics are the same as the COSTAR husky which has already been modeled.  We were unable to perform additional validation tests due to COVID-19 restrictions.  

### Endurance Characteristics
This configuration has an endurance of approximately 4 hours, but we have limited the model in simulation to have a 2 hour endurance.  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.  

### Diversions from Physical Hardware of MARBLE Husky
Computers were installed in the payload area of the husky and these have been roughly modeled (the rail and computer bay are shown in the model.sdf model).  The MARBLE vehicle uses an AMD Ryzen processor (32-core) with 64 GB of RAM.  It has a cooling system as well.  

## Marble Husky Validation and Specification Links
* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
  * FLIR Blackfly Camera (HD MIPI Cameras) - https://www.edmundoptics.com/p/bfly-pge-05s2c-cs-13-blackflyreg-poe-gige-color-camera/30315/
    - With lens: https://www.bhphotovideo.com/c/product/414239-REG/Tamron_13FM28IR_13FM28IR_2_8_mm_f_1_2.html
  * Ouster 3D Lidar (64 Channel) - https://ouster.com/products/os1-lidar-sensor/
  * RPLidar S1 Planar Lidar - https://www.slamtec.com/en/Lidar/S1Spec 
  * IMU: Microstrain 3DM-GX5-25 - datasheet: https://www.microstrain.com/sites/default/files/applications/files/3dm-gx5-25_datasheet_8400-0093_rev_n.pdf
    * Explanation of sensor parameter derivations:
	We derived the stddev terms as follows:

	accelerometer noise density = 0.00002 g/sqrt(Hz)
		=> convert to m/s^2 => 1.962e-4 m/s^2
	gyro noise density = 0.005 deg/s/sqrt(Hz)
		=> convert to rad/sec => 8.72664e-5 radians

	Other terms are difficult to extract from datasheet, so we used similar terms to previous IMU models proposed (of similar or worse quality) such as the ADIS 16448 (which has worse performance than this IMU).

* \<Validation Video Link(s), e.g.,https://youtu.be/xxxxxxxxx/\>
* \<Validation Data Link(s), e.g., https://drive.google.com/file/xxxxxxxxx/\>

We are unable to provide validation video links and data links at this time due to the COVID-19 safety measures preventing us from accessing the necessary lab resources.  
