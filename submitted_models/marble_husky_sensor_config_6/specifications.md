<!---This is a Markdown description of a robot model submitted for inclusion in the DARPA Subterranean Challenge Technology Repository -->

# MARBLE Husky Sensor Config 6
This specifications.md file is a description and proof of virtual model validation for the MARBLE Husky with Sensor Configuration 6. This robot may be launched using an ign launch command with the variable name `marble_husky_sensor_config_6`.

## Description
This configuration is based on Clearpath Robotics Husky ground robot. The marble sensor suite is located at the front of the husky and includes 2 3D LIDARs, 1 Planar LIDAR, 4 RGB Cameras, and 2 Picco Flexx TOF cameras  

## Usage Instructions
The robot can be used in the same manner as the COSTAR Husky robot.  It accepts twist messages on the cmd_vel topic.  

## Usage Rights
The same Rights are granted for the configuration as for the COSTAR Husky. No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The MARBLE Husky has the following estimate commercial cost components:
* Base vehicle: $25,000
* Sensor suite: $18,700
* Compute/Support Electronics: $4,000
* Total: ~ $47,700

Sensor Suite Price Breakdown: 
RPLidar S1 = $650 * 1
FLIR Backly + Lens = $312.50 * 4 
Ouster OS1 - Gen 1 = $8000 * 2 (Requested price catalog)
Picco Flexx TOF Cameras = $400*2 
Total = $18700

Its weight is approximately 125 lbs (two-person heft).

### Sensors
This MARBLE Husky with sensor configuration 1 includes the following sensors. The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Picco Flexx TOF Cameras (x2)
* FLIR Backfly PoE GigE Color Camera Camera (x4), modeled by rgb camera plugin
  - Fixed on front of vehicle, angled left and right slightly
* Ouster 3D Lidars (x2), modeled by gpu_lidar plugin
- RPLidar S1 Planar Lidar (under the 3D lidar), modeled by gpu_ray plugin

### Control
This MARBLE Husky is controlled by the DiffDrive plugin.  It accepts twist inputs which drive the vehicle along the x-direction and around the z-axis.  

### Motion CharacteristicsBased on the tests specified in the DARPA SubT Challenge [Model PreparationGuide](https://subtchallenge.com/\<fix_me\>), this vehicle has the following motion constraint characteristics.

This configuration has the same motion characteristics as the COSTAR husky.
The motion characteristics are the same as the COSTAR husky which has already been modeled. 
Validation data is forthcoming.

### Endurance Characteristics
Validation data is forthcoming.

### Diversions from Physical Hardware of MARBLE Husky
Computers were installed in the payload area of the husky and these have been roughly modeled (the rail and computer bay are shown in the model.sdf model).  The MARBLE vehicle uses an AMD Ryzen processor (32-core) with 64 GB of RAM.  It has a cooling system as well.  

## X4 Validation and Specification Links

* \<Validation Video Link(s), e.g.,https://youtu.be/xxxxxxxxx/\>
* \<Validation Data Link(s), e.g., https://drive.google.com/file/xxxxxxxxx/\>

We are unable to provide validation video links and data links at this time due to the COVID-19 safety measures preventing us from accessing the necessary lab resources.  

* Sensor specification links:
  * FLIR Backfly Camera - https://flir.app.boxcn.net/s/iicqenjhtth41dt13951qh5toidx29ih + lens --> https://www.bhphotovideo.com/c/product/414239-REG/Tamron_13FM28IR_13FM28IR_2_8_mm_f_1_2.html
  * Ouster 3D Lidar (64 Channel) - https://ouster.com/products/os1-lidar-sensor/
  * RPLidar S1 Planar Lidar - https://www.slamtec.com/en/Lidar/S1Spec
  * Picco Flexx Camera - https://www.automation24.com/development-kit-pmd-vision-r-camboard-pico-flexx-700-000-094


