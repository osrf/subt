<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE QAV500 Sensor Config 3
This specifications.md file is a description and proof of virtual model validation for the MARBLE QAV500 with Sensor Configuration 3. This robot may be launched using an ign launch command with the variable name `marble_qav500_sensor_config_3`.


## Description
This configuration is based on the Lumineer QAV500 Quadcopter. The marble sensor suite is located at the front and bottom of the chassis.

## Usage Instructions
The robot can be used in the same by sending Twist commands to the vehicle_name/cmd_vel ROS topic.

## Usage Rights
The same Rights are granted for the configuration as for the MARBLE Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE QAV500 frame has the same estimated commercial cost as the QAV500 vehicle of $200, plus additional costs associated with the sensors ($10,000). Its weight is approximately 6.6 lbs (possible to heft with a single person).

### Sensors
This QAV500 with sensor configuration 3 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* D435i RGBD Camera, modeled by rgbd_camera plugin
  - 1x fixed, forward-looking
* Ouster 3D Lidar (64 Channel), modeled by gpu_lidar plugin
* Microstrain IMU: 3DM-GX5-15, modeled by imu_sensor plugin. Notes on modeling of the IMU are included in the model.sdf file.
* Picoflexx TOF Cameras, modeled by depth_camera plugin
  - 1 fixed on the front top
  - 1 fixed on the front bottom
* VGA 640x480 RGBD cameras
  - 2x fixed, facing either side and angled down


### Control
This vehicle is controlled by the Twist ROS topic cmd_vel

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics.

This configuration has the similar motion characteristics as the X4 vehicle due to the size, except it also has different mass characteristics. More detailed characteristics are not available due to the coroavirus crisis.

### Endurance Characteristics
This configuration has an endurance of approximately 12 minutes or 720 seconds  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.

### Diversions from Physical Hardware of \<Robot Name\> <Explanation of Diversions\>
Computers were installed in the payload area of the QAV500 and these have been roughly modeled.  The MARBLE vehicle uses an Intel Nuc with Core i7-8650U processor.

## <a name="validation_links"></a>X4 Validation and Specification Links
* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
  * Ouster 3D Lidar (64 Channel) - https://ouster.com/products/os1-lidar-sensor/
  * Picoflexx TOF camera (depth image and point cloud) - https://pmdtec.com/picofamily/wp-content/uploads/2018/03/PMD_DevKit_Brief_CB_pico_flexx_CE_V0218-1.pdf
  * IMU: Microstrain 3DM-GX5-15 - datasheet: https://www.microstrain.com/sites/default/files/applications/files/3dm-gx5-15_datasheet_8400-0094_rev_m.pdf
    * Explanation of sensor parameter derivations:
	We derived the stddev terms as follows:

	accelerometer noise density = 0.00002 g/sqrt(Hz)
		=> convert to m/s^2 => 1.962e-4 m/s^2
	gyro noise density = 0.005 deg/s/sqrt(Hz)
		=> convert to rad/sec => 8.72664e-5 radians

	Other terms are difficult to extract from datasheet, so we used similar terms to previous IMU models proposed (of similar or worse quality) such as the ADIS 16448 (which has worse performance than this IMU).

* \<Validation Video Link(s), e.g.,https://youtu.be/xxxxxxxxx/\>
* \<Validation Data Link(s), e.g., https://drive.google.com/file/xxxxxxxxx/\>

We are unable to provide validation video links and data links at this time due to the COVID-19 safety measures preventing us from accessing the necessary lab resources at this time.
