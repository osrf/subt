<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE HD2 Sensor Config 3
This specifications.md file is a description and proof of virtual model validation for the MARBLE HD2 with Sensor Configuration 3. This robot may be launched using an ign launch command with the variable name `MARBLE_HD2_SENSOR_CONFIG_3`.

## Description
This configuration is based on Superdroid HD2 ground robot. The marble sensor suite is located at the front of the HD2 chassis and includes stationary sensors (cameras, lidars, etc).  

## Usage Instructions
The robot can be used in the same manner as the MARBLE Husky robot.  

## Usage Rights
The same Rights are granted for the configuration as for the MARBLE Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE HD2 has the following estimated commercial costs:
* Base vehicle: $10,000
* Sensor Suite: $14,000
* Compute/support electronics: $4,000
* Total: ~ $28,000

Its weight is approximately 75 lbs (possible, but heavy to heft with a single person). 

### Sensors
This HD2 with sensor configuration 3 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#Validation-Links) section.

The following specific sensors are declared payloads of this vehicle:

* D435i RGBD Camera, modeled by rgbd_camera plugin
    - fixed, downward-facing at about 45 degrees (for examining terrain)
* Ouster 3D Lidar (64 Channel), modeled by gpu_lidar plugin
* Microstrain IMU: 3DM-GX5-25, modeled by imu_sensor plugin. Notes on modeling of the IMU are included in the model.sdf file. (located under the 3D lidar, installed at same x,y location as 3D lidar)
* RPLidar S1 Planar Lidar (under the 3D lidar), modeled by gpu_ray plugin
* HD MIPI Cameras - 720p (x2)
    - (1x) forward facing on top of sensor tower
    - (1x) rear facing on top of sensor tower

### Control
This MARBLE HD2 is controlled by the DiffDrive plugin.  It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  We add additional pseudo-wheels where the HD2's treads are to better approximate a track vehicle.  Currently, we are not aware of a track-vehicle plugin for ignition-gazebo.  A TrackedVehicle plugin does exist in gazebo8+, but it is not straightforward to port to ignition-gazebo.  We hope to work with other SubT teams and possibly experts among the ignition-gazebo developers to address this in the future.  

### Motion Characteristics Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://github.com/osrf/subt/wiki/Model%20Submission%20Process), this vehicle has the following motion constraint characteristics. 

This configuration has roughly the same motion characteristics as the COSTAR/MARBLE husky vehicles.

At this time, we assume that the motion characteristics are the same as the COSTAR husky which has already been modeled.  We were unable to perform additional validation tests due to COVID-19 restrictions.  

### Endurance Characteristics
This configuration has an endurance of approximately 2 hours.  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.  

### Diversions from Physical Hardware of MARBLE HD2
Computers were installed in the payload area of the husky and these have been roughly modeled (the rail and computer bay are shown in the model.sdf model).  The MARBLE vehicle uses an AMD Ryzen processor (32-core) with 64 GB of RAM.  It has a cooling system as well which makes up part of the black bay seen in the sdf model.  

## Validation Links
* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
  * (2x) Ouster 3D Lidar (64 Channel) - https://ouster.com/products/os1-lidar-sensor/ (one vertically oriented and another horizontally oriented)
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
