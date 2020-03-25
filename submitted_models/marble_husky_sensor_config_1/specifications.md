<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE Husky Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the MARBLE Husky with Sensor Configuration 1. This robot may be launched using an ign launch command with the variable name `marble_husky_sensor_config_1`.

## Description
This configuration is based on Clearpath Robotics Husky ground robot. The marble sensor suite is located at the front of the husky and includes stationary sensors and some sensors and a light on a pan/tilt mechanism.  

## Usage Instructions
The robot can be used in the same manner as the COSTAR Husky robot.

## Usage Rights
The same Rights are granted for the configuration as for the COSTAR Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE Husky has the same estimated commercial cost as the Husky vehicle, plus additional costs associated with the sensors. Its weight is approximately 80 lbs (two-person heft). 

### Sensors
This \<Robot Name\> with sensor configuration \<Sensor Config #\> includesthe following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:* \<Sensor Category 1\> -\<Sensor 1\>, modeled by \<Sensor 1 Plugin\>* ...* \<Sensor Category n\> -\<Sensor n\>, modeled by \<Sensor n Plugin\>

### Control
This \<Robot Name\> is controlled by the (open-source/custom) \<Control Software\> package.

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics. 

This configuration has the same motion characteristics as the COSTAR husky, except it also has a pan/tilt mechanism which has additional motion characteristics.  We have included a test script in this folder (test_gimbal.sh) which sends pan and tilt commands using ROS topics.  This script assumes you have named your vehicle X1.  

### Endurance Characteristics
This configuration has an endurance of approximately 4 hours, but we have limited the model in simulation to have a 2 hour endurance.  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.  

### Diversions from Physical Hardware of \<Robot Name\> <Explanation of Diversions\>

## <a name="validation_links"></a>X4 Validation and Specification Links
* \<Vehicle Specification Link(s)\>
* \<Sensor Specification Link(s)\>
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

We are currently unable to provide validation data links due to coronavirus spreading prevention measures at CU Boulder and SSCI.  
