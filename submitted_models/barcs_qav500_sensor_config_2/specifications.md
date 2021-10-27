<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# BARCS QAV500 Sensor Config 2
This specifications.md file is a description and proof of virtual model validation for the BARCS MARBLE QAV500 with Sensor Configuration 2. This robot may be launched using an ign launch command with the variable name `barcs_qav500_sensor_config_2`.

## Description
This configuration is based on the Lumineer QAV500 Quadcopter. The barcs sensor suite is located at the front and bottom of the chassis.

## Usage Instructions
The robot can be used in the same by sending Twist commands to the vehicle_name/cmd_vel ROS topic.

## Usage Rights
The same Rights are granted for the configuration as for the MARBLE Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE QAV500 frame has the same estimated commercial cost as the QAV500 vehicle of $200, plus additional costs associated with the sensors. Its weight is approximately 6.6 lbs (possible to heft with a single person).

### Sensors
This QAV500 with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Blackfly S camera, modeled by camera plugin
  - 1x fixed, forward-looking
* A Velodyne VLP-16 Lidar, modeled by the gpu_lidar plugin
* Microstrain IMU: 3DM-GX5-15, modeled by imu_sensor plugin. Notes on modeling of the IMU are included in the model.sdf file.
* 2x LIDAR-Lite point lidars, modeled by gpu_ray plugin
  - 1 fixed on the front top
  - 1 fixed on the front bottom


### Control
This vehicle is controlled by the Twist ROS topic cmd_vel

### Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_, _y_, and _z_ velocity range from -4.0 m/s to 4.0 m/s
* _x_ and _y_ acceleration range from -5.0 m/s<sup>2</sup> to 5.0 m/s<sup>2</sup>
* _z_ acceleration range from -3.0 m/s<sup>2</sup> to 3.0 m/s<sup>2</sup>

This configuration has the similar motion characteristics as the X4 vehicle due to the size, except it also has different mass characteristics. More detailed characteristics are not available due to the coronavirus crisis.

### Endurance Characteristics
This configuration has an endurance of approximately 12 minutes or 720 seconds, but we have limited the model in simulation to have 10 minute endurance.

### Diversions from Physical Hardware of MARBLE QAV500
Computers were installed in the payload area of the QAV500 and these have been roughly modeled.  The MARBLE vehicle uses an Intel Nuc with Core i7-8650U processor.

The endurance and motion characteristics above are approximated to match existing models until model validation data can be provided.

## <a name="validation_links"></a>Validation and Specification Links
* Vehicle Links:
  * https://www.getfpv.com/qav500-v2-fpv-quadcopter.html

* Sensor specification links:
  * [LIDAR - Velodyne VLP-16](https://www.amtechs.co.jp/product/VLP-16-Puck.pdf)
  * Picoflexx TOF camera (depth image and point cloud) - https://pmdtec.com/picofamily/wp-content/uploads/2018/03/PMD_DevKit_Brief_CB_pico_flexx_CE_V0218-1.pdf
  * Color Camera - FLIR - Blackfly S USB3 Model ##BFS-U3-16S2C-CS: [Link](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS)  
  * LIDAR-Lite - https://www.garmin.com/en-US/p/578152#specs
  * IMU: Microstrain 3DM-GX5-15 - datasheet: https://www.microstrain.com/sites/default/files/applications/files/3dm-gx5-15_datasheet_8400-0094_rev_m.pdf
