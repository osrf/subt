<!-- This is a Markdown description of a robot model submitted for inclusion in the
DARPA Subterranean Challenge Technology Repository -->

# COSTAR Shafter Sensor Config 1
<div style="text-align: justify">
This specifications.md file is a description and proof of virtual model validation for the COSTAR Shafter with Sensor Configuration 1. This robot may be launched using an `ign launch` command with the variable name `costar_shafter_sensor_config_1`. </div>

## Description
<div style="text-align: justify">
The shafter aerial platform is an autonomous system capable of being deployed in an unknown environment and proceeding for its mapping up to the point of automatic return to home. It performs localization, planning, control, mapping and artifact localization tasks based on the onboard sensing and algorithm design. In this configuration, the platform is equipped with the following sensors: 3D LIDAR, forward facing RGBD camera and an IMU. </div>

## Usage Instructions
To launch the aerial platform in the simulation environment use the following commands:
```
cd (path to package)/launch
ign launch -v 4 example.ign robotName:=shafter modelName:=costar_shafter_sensor_config_1
```

The robot can be controlled by sending Twist commands to the vehicle_name/cmd_vel ROS topic.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

### Cost and Scale
The COSTAR Shafter robot has an estimated cost of $8.000 and weighs around 3.1 kg.

### Sensors
This COSTAR Shafter with sensor configuration 1 includes the following sensors. 

* 1x IMU - Cube Module, modeled by `imu` plugin
* 1x 3D LIDAR - Velodyne VLP-16, modeled by `gpu_ray` plugin
* 1x RGBD Camera - Realsense ZR300, modeled by `rgbd_camera` plugin

### Control
This COSTAR Shafter platform is controlled by the default twist controller package inside the simulation environment.

### Motion Characteristics
<div style="text-align: justify">
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics. </div>

* _x_ linear velocity range from -2.0 m/s to 2.0 m/s
* _y_ linear velocity range from -1.95 m/s to 1.95 m/s
* _z_ linear velocity range from -1.0 m/s to 1.0 m/s
* _x_ linear acceleration range from -3.57 m/s<sup>2</sup> to 3.57 m/s<sup>2</sup>
* _y_ linear acceleration range from -3.66 m/s<sup>2</sup> to 3.66 m/s<sup>2</sup>
* _z_ linear acceleration range from -1.5 m/s<sup>2</sup> to 1.5 m/s<sup>2</sup>
* _x_ angular velocity range from -5.0 rad/s to 5.0 rad/s
* _y_ angular velocity range from -4.55 rad/s to 4.55 rad/s
* _z_ angular velocity range from -3.6 rad/s to 3.6 rad/s

The constraints can be found in the following locations within the simulation model:

* spawner.rb, lines 112-114

### Endurance Characteristics
<div style="text-align: justify">
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following endurance characteristics. </div>

* Battery life of 600 seconds

### Diversions from Physical Hardware of COSTAR Shafter Sensor Config 1

* Controller in the simulation model is different from the one in the real platform. 
* The real platform uses an Intel Nuc with Core i5-10210U Processor.
* The current version of real platform does not carry extra illumination, it is planned to be added in the future. Nevertheless, the virtual platform is equipped with an led light for simulation purposes.

## <a name="validation_links"></a>COSTAR Shafter Sensor Config 1 Validation and Specification Links

* IMU - Cube Module IMU: [Link](https://www.robotshop.com/eu/en/cubepilot-the-cube-black-set-standard-carrier.html)
* 3D LIDAR - Velodyne - VLP-16 PuckLite: [Link](https://velodynelidar.com/products/puck-lite/)
* RGBD - Realsense - ZR300: [Link](https://docs.rs-online.com/5f51/0900766b815b44f8.pdf)
* Linear Acceleration and Speed Validation Video: [Link](https://drive.google.com/file/d/1M53pMNR44Ueb_CtLH7ljEZJuyrnibWPD/view?usp=sharing)
* Linear Acceleration and Speed Validation Data: [Link](https://drive.google.com/drive/folders/1z2chKm4AGy8hZsBgSIs66Btexv78O9km?usp=sharing)
* Angular Acceleration and Speed Validation Video: [Link](https://drive.google.com/file/d/1ZunfZfD0XDU1xCZYNJq0JbRrnMYEAqOt/view?usp=sharing)
* Angular Acceleration and Speed Validation Data: [Link](https://drive.google.com/drive/folders/1z2chKm4AGy8hZsBgSIs66Btexv78O9km?usp=sharing)
* Battery Endurance Validation Video: [Link](https://drive.google.com/file/d/1Mv1udI9n3u5hF4Thffh4AG0-j6y3ZUFE/view?usp=sharing)
* Battery Endurance Validation Data: [Link](https://drive.google.com/drive/folders/1z2chKm4AGy8hZsBgSIs66Btexv78O9km?usp=sharing)
