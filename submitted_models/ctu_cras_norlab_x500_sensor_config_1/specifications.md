# CTU-CRAS-NORLAB-X500
This specifications.md file is a description and proof of virtual model validation for CTU-CRAS-NORLAB's robot X500 with Sensor Config 1. 
This robot may be launched using the `ign launch ` command with the variable name `ctu_cras_norlab_x500_sensor_config_1`.

## Description
X500 is a quadrotor UAV based on the Holybro X500 frame built for the Final Circuit of the DARPA Subterranean Challenge.
Its sensory equipment consisting of 3D LiDAR, 2 RGBD cameras and 1 RGB camera allows reliable navigation through various environments, its small size facilitates flying through narrow passages.
A two-battery system consisting of 2 Lipo 6750mAh 4-cell batteries connected in parallel can power the UAV for over 25 minutes of flight time.
The propulsion system consists of T-MOTOR MN3510 KV700 motors and 13 inch propellers.

## Usage Instructions
The robot motion is controlled via standard cmd_vel commands.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

## Cost and Scale
The CTU-CRAS-NORLAB's X500 UAV  has an estimated commercial cost of 12500 USD.
It weights approximately 3.3 kg.

## Sensors
This X500 with sensor configuration Sensor Config 1 includes the following sensors. 
The specifications of the sensors are provided below in
the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 2x RGBD camera - Intel Realsense D435i, modeled by `rgbd_camera` plugin.
* 1x 3D medium range LiDAR - Ouster OS0-64, modeled by `gpu_lidar` plugin.
* 1x HD wide camera - Basler dart daA1600-60um with DSL165A lens, modeled by `camera` plugin.
* 1x IMU with standard SubT specification, modeled by `imu` plugin.

### Basler HD wide camera field of view

* horizontal sensor size x = 7.2 mm
* lens focal length f = 4.57 mm
* HFoV = 2 * atan((x/2)/f) = 1.3344 rad = 76.2 deg
* HFoV with the camera and lens on our [testbed](https://photos.google.com/u/1/share/AF1QipNFt3voXyQMq9E2hoaKImq9K2o2vBZdUgWQtd9WmCxNMdLBiPcG-QrDDGrBWJf5sQ?key=RVg1cjk0dGxXSF95Q1poXzRCd3ZESmZ1OEtxWUtR) is 97.47 deg

## Control
This X500 is controlled by the open-source [mrs_uav_controllers](https://github.com/ctu-mrs/mrs_uav_controllers) package. 

## Motion Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following motion constraint characteristics:

* _x_ velocity range from -8.61 m/s to 8.61 m/s
* _y_ velocity range from -8.6 m/s to 8.6 m/s 
* _z_ velocity range from -4.47 m/s to 4.47 m/s.
* _x_ acceleration range from -7.49 m/s<sup>2</sup> to 7.49 m/s<sup>2</sup>.
* _y_ acceleration range from -8.13 m/s<sup>2</sup> to 8.13 m/s<sup>2</sup>.
* _z_ acceleration range from -7.29 m/s<sup>2</sup> to 7.29 m/s<sup>2</sup>.
* _z_ angular velocity range from -2.44 rad/s to 2.44 rad/s.

The constraints can be found in the following locations within the simulation model package:

* `spawner.rb`, lines 113-115

## Endurance Characteristics
Based on the tests specified in the DARPA SubT Challenge [Model Preparation Guide](https://subtchallenge.com/resources/Simulation_Model_Preparation_Guide.pdf), this vehicle has the following endurance characteristics:
* Battery life of 1500 seconds.

## Diversions from Physical Hardware
Virtual X500 is a faithful representation of real-world X500 with few diversions:
* IMU:
  * Real-world: we are using the IMU from [Pixhawk4 FCU](https://docs.px4.io/master/en/flight_controller/pixhawk4.html#quick-summary)
  * Virtual: IMU from the [subt repository](https://github.com/osrf/subt/wiki/api#sensors)

## <a name="validation_links"></a>Validation and Specification Links
* Vehicle specification links:
  * [X500 frame](http://www.holybro.com/product/x500-kit/)
  * [Motors](https://store-en.tmotor.com/goods.php?id=339)
  * [Batteries](https://www.professional-multirotors.com/product/tattu-6750mah-14-8v-25c-4s1p-lipo-battery-99wh/)
* Sensor specification links:
  * [Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/)
  * [Basler dart daA1600-60um](https://www.baslerweb.com/en/products/cameras/area-scan-cameras/dart/daa1600-60um-s-mount/)
  * [DSL165a lens](https://www.framos.com/en/dsl165a-nir-f1.6-22853)
  * [Ouster OS0-64 LiDAR](https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p0-os0.pdf)
* Motion validation links
  * [Unedited video](https://www.youtube.com/watch?v=ilg0HgWysHg)
  * [Video with visualized telemetry](https://www.youtube.com/watch?v=w_62XWc6W7w)
  * [Telemetry data](https://nasmrs.felk.cvut.cz/index.php/s/80oKJh506PsmbNQ)
* Endurance validation links
  * [Unedited video](https://www.youtube.com/watch?v=HlTlnZcfB7I)
