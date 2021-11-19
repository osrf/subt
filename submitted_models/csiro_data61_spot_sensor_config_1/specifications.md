# CSIRO-Data61 Boston Dynamics Spot Configuration
This specifications.md file is a description and proof of virtual model validation for
the CSIRO_DATA61 SPOT with Sensor Configuration 1. This robot may be launched using an `ign launch` command with the
`robotConfigN` variable name `CSIRO_DATA61_SPOT_SENSOR_CONFIG_1`.

Refer to [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md) for information about the robot base.

## Description
This sensor config incorporates a standardised perception payload consisting of a gimbal mounted lidar, imu, 4 RGB cameras and a gas sensor.

## Usage Instructions

For information regarding controlling the platform see [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

The robot accepts standard twist commands on the cmd_vel topic. The gimbal for the lidar can be controlled on the following topic:
lidar_gimbal/pan_rate_cmd_double (std_msgs/Double)

The position of the gimbal is accessed on the joint state topic.

## Usage Rights
This software is released under a [BSD 3-Clause license](LICENSE).

## Cost and Scale
* Base vehicle: $75,000
* Platform weight is approximately 37.82Kg with payload and battery

## Sensors
This sensor configuration of Spot includes the base sensors described in[bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md). The robot has the following additional sensors:

* A Velodyne VLP-16 Lidar, modeled by the gpu_lidar plugin. (note this lidar is also mounted at 45 degrees on a rotating gimbal)
* ECON e-CAM130_CUXVR Quad Camera system with each camera mounted on one side of the payload. The version on the platform is a custom set of sensors run at a resolution of 2016x1512, hardware triggered at 15fps. They are modeled by the standard camera plugin
* A Microstrain CV5-25 IMU, modeled by the standard imu plugin
* Simulation gas sensor

The specifications for these instruments are provided below

## Control

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

## Motion characteristics

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
The 6.12 kg sensory backpack does not significantly affect the motion characteristics of the base platform.

## Endurance Characteristics
This configuration has an endurance of approximately 72 minutes

## Diversions from Physical Hardware of Spot robot
See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

## Changing parameters of the robot
After changing the xacro configuration of the robot, or if changes are made to the base platform (bosdyn_spot), run script `scripts/update_robot_sdf` to write the
changes to the `model.sdf` file. This updates the SDF just for this particular robot. After the update script is called, review the resulting SDF and
commit the changes.

__Do not change the SDF file directly, as the changes could be lost the next time the update script is run.__

The URDF files are not physically present, but are generated on-the-fly by calling script
`scripts/print_robot_urdf`.

## Validation and Specification Links
* Platform Validation Links
  * See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
  * [Endurance Validation Video](https://youtu.be/XXc2f_pidoU).
  * [Endurance Validation Data](https://drive.google.com/file/d/1JMx5tlTDgh_xuqIHYSgwyui8Yp8Uf6fh/view?usp=sharing).
* Sensor specification links:
  * [LIDAR - Velodyne VLP-16](https://velodynelidar.com/products/puck/)
  * [IMU - Microstrain CV5-25](https://www.microstrain.com/inertial/3dm-cv5-25)
  * [Cameras - ECON e-CAM130_CUXVR ](https://www.e-consystems.com/nvidia-cameras/jetson-agx-xavier-cameras/four-synchronized-4k-cameras.asp)