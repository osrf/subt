<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE HD2 Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the MARBLE HD2 with Sensor Configuration 1. This robot may be launched using an ign launch command with the variable name `marble_hd2_sensor_config_1`.

## Description
This configuration is based on Superdroid HD2 ground robot. The marble sensor suite is located at the front of the HD2 chassis and includes stationary sensors and some sensors and a light on a pan/tilt mechanism.

## Usage Instructions
The robot can be used in the same manner as the MARBLE Husky robot.  The gimbal can be controlled using the test_gimbal.sh script (in this folder).  There are also scripts to print the current angles from the pan and tilt axes of the gimbal (pan_echo.sh and tilt_echo.sh).

The gimbal can be controlled via the following topics:
pan_tilt/pan_rate_cmd_double (send a std_msgs/Double message)
pan_tilt/tilt_rate_cmd_double (send a std_msgs/Double message)

The position of the pan and tilt axes can be accessed through the joint_state topic.  The name of each axis is available inside that message, but as of today the pan and tilt can be accessed as follows:
Pan: joint_state/position[4]
Tilt: joint_state/position[5]

## Usage Rights
The same Rights are granted for the configuration as for the MARBLE Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE HD2 has the following estimated commercial costs:
* Base vehicle: $10,000
* Sensor Suite: $12,000
* Compute/support electronics: $4,000
* Total: ~ $26,000

Its weight is approximately 75 lbs (possible, but heavy to heft with a single person).

### Sensors
This HD2 with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Trossen ScorpionX MX-64 Robot Turret, modeled by JointStateController and JointStatePublisher plugins.
* D435i RGBD Camera (x3), modeled by rgbd_camera plugin
  - 1x fixed, forward-looking
  - 1x fixed, downward-facing at about 45 degrees (for examining terrain)
  - 1x gimballed (on the Trossen turret)
* Ouster 3D Lidar (64 Channel), modeled by gpu_lidar plugin
* Microstrain IMU: 3DM-GX5-25, modeled by imu_sensor plugin. Notes on modeling of the IMU are included in the model.sdf file.  (located under the 3D lidar, installed at same x,y location as 3D lidar)
- RPLidar S1 Planar Lidar (under the 3D lidar), modeled by gpu_ray plugin
- Vividia HTI-301 LWIR Camera (not modeled because thermal camera not yet supported in simulator) - located on the turret next to the D435i and light.

### Control
This MARBLE HD2 is controlled by the DiffDrive plugin.  It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  We add additional pseudo-wheels where the HD2's treads are to better approximate a track vehicle.  Currently, we are not aware of a track-vehicle plugin for ignition-gazebo.  A TrackedVehicle plugin does exist in gazebo8+, but it is not straightforward to port to ignition-gazebo.  We hope to work with other SubT teams and possibly experts among the ignition-gazebo developers to address this in the future.

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics.

This configuration has roughly the same motion characteristics as the COSTAR/MARBLE husky vehicles, except it also has a pan/tilt mechanism which has additional motion characteristics.  We have included a test script in this folder (test_gimbal.sh) which sends pan and tilt commands using ROS topics.  This script assumes you have named your vehicle X1.

At this time, we assume that the motion characteristics are the same as the COSTAR husky which has already been modeled.  We were unable to perform additional validation tests due to COVID-19 restrictions.

### Endurance Characteristics
This configuration has an endurance of approximately 2 hours.  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.

### Diversions from Physical Hardware of MARBLE HD2
Computers were installed in the payload area of the husky and these have been roughly modeled (the rail and computer bay are shown in the model.sdf model).  The MARBLE vehicle uses an AMD Ryzen processor (32-core) with 64 GB of RAM.  It has a cooling system as well which makes up part of the black bay seen in the sdf model.

## X4 Validation and Specification Links
* Vehicle Links:
  * https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

* Sensor specification links:
  * Trossen ScorpionX MX-64 Robot Turret - https://www.trossenrobotics.com/p/ScorpionX-RX-64-robot-turret.aspx
  * D435i RGBD Camera - https://www.intelrealsense.com/depth-camera-d435i/
  * Ouster 3D Lidar (64 Channel) - https://ouster.com/products/os1-lidar-sensor/
  * RPLidar S1 Planar Lidar - https://www.slamtec.com/en/Lidar/S1Spec
  * Vividia HTI-301 LWIR Camera - https://www.oasisscientific.com/store/p504/Vividia_HTi_HT-301_Thermal_Imaging_Camera_for_Android_Phone_and_Tablet_with_IR_Resolution_384x288.html
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
