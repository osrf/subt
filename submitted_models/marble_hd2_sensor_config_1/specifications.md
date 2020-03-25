<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE HD2 Sensor Config 1
This specifications.md file is a description and proof of virtual model validation for the MARBLE HD2 with Sensor Configuration 1. This robot may be launched using an ign launch command with the variable name `marble_hd2_sensor_config_1`.

## Description
This configuration is based on Superdroid HD2 ground robot. The marble sensor suite is located at the front of the HD2 chassis and includes stationary sensors and some sensors and a light on a pan/tilt mechanism.  

## Usage Instructions
The robot can be used in the same manner as the MARBLE Husky robot.  The gimbal can be controlled using the test_gimbal.sh script (in this folder).  There are also scripts to print the current angles from the pan and tilt axes of the gimbal (pan_echo.sh and tilt_echo.sh).  

## Usage Rights
The same Rights are granted for the configuration as for the MARBLE Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE HD2 has the same estimated commercial cost as the HD2 vehicle, plus additional costs associated with the sensors. Its weight is approximately 75 lbs (possible to heft with a single person). 

### Sensors
This HD2 with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

Front payload sensors use the same setup as the MARBLE Husky platform.  The following sensors are included:

* \<Sensor Category 1\> -\<Sensor 1\>, modeled by \<Sensor 1 Plugin\>* ...* \<Sensor Category n\> -\<Sensor n\>, modeled by \<Sensor n Plugin\>

### Control
This \<Robot Name\> is controlled by the (open-source/custom) \<Control Software\> package.

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics. 

This configuration has the same motion characteristics as the COSTAR husky, except it also has a pan/tilt mechanism which has additional motion characteristics.  We have included a test script in this folder (test_gimbal.sh) which sends pan and tilt commands using ROS topics.  This script assumes you have named your vehicle X1.  

### Endurance Characteristics
This configuration has an endurance of approximately 4 hours, but we have limited the model in simulation to have a 2 hour endurance.  We plan to carry out the endurance test characterization but have been prevented from doing so due to the coronavirus measures preventing us from visiting the lab space while preparing these models for simulation.  

### Diversions from Physical Hardware of \<Robot Name\> <Explanation of Diversions\>

## <a name="validation_links"></a>X4 Validation and Specification Links
* \<Vehicle Specification Link(s)\>* \<Sensor Specification Link(s)\>* \<Validation Video Link(s), e.g.,https://youtu.be/xxxxxxxxx/\>* \<Validation Data Link(s), e.g., https://drive.google.com/file/xxxxxxxxx/\>

We are currently unable to provide validation data links due to coronavirus spreading prevention measures at CU Boulder and SSCI.  
