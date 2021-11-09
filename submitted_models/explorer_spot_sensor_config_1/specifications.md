# Boston Dynamics Spot - Explorer sensor config 1
This specifications.md file is a description and proof of virtual model validation for the
Boston Dynamics Spot robot configured by team Explorer. This robot may be launched using an
ign launch command with the variable name `explorer_spot_sensor_config_1`.

For local testing, you can use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/explorer_spot_sensor_config_1/launch/example.ign robotName:=X1 ros:=true champ:=true

This specifications.md file describes mainly the additional sensory payload of our Spot. Refer to
[bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md) for information about the robot base,
how to control it and how to modify parameters of the robot.

## Description
This sensor config carries 4 wide-angle RGBD cameras, an IMU and a 3D lidar.

## Usage Instructions

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md). There is no diversion from
the principles described there, just substitute `bosdyn_spot` with `explorer_spot_sensor_config_1`.

## Usage Rights
No additional restrictions have to be taken into account for this model.

## Cost and Scale
* Base vehicle: $75,000
* RGBD Cameras: $2,500
* IMU: $1,500
* Computers: $1,500
* Lidar: $5,000
* Total price: $85,500

vehicle weight: 40kg (37kg base mass + 3kg sensor payload)

## Sensors
This sensor configuration of Spot includes the following sensors in addition to the base sensors of the platform
described in [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* 1x 3D medium range lidar &mdash; Velodyne-16, modeled by `gpu_lidar` plugin.
* 4x RGBD camera &mdash; intel realsense depth d435i, modeled by `rgbd_camera` plugin.
* 1x IMU &mdash; Xsense MTI-100, modeled by `imu` plugins.

## Control

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

## Motion characteristics

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
The 4 kg sensory backpack is not expected to affect the maximum speed of the robot
as it keeps low center of gravity and is well under the maximum declared payload weight (11 kg).

## Endurance Characteristics
This configuration has an endurance of approximately 75 minutes (value from datasheet corrected
by consumption of the sensors and computers).

## Diversions from Physical Hardware of Spot robot
See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

Cameras have no distortion as the simulator doesn't support it.

The Velodyne lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

## Changing parameters of the robot
Most parameters affecting the simulation model performance are in the `config/model` folder,
in files `common.yaml` and `ign.yaml`. If some parameter is not exposed to these config files,
look into the Xacro files in `urdf/` folder and change the values there. Also look in the respective
config folder of the `bosdyn_spot` base model. Values in configs in this package override the values
set in the base model.

After changing a value in these files, run script `scripts/update_robot_sdf` to write the
changes to the `model.sdf` file. This updates the SDF just for this particular robot. It is also needed
to call it if the base robot model has changed (`bosdyn_spot`).

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

The URDF files are not physically present, but are generated on-the-fly by calling script
`scripts/print_robot_urdf`.

The color of the payload on spot is directly defined in `model.sdf` from line 390-394. After running update script,
users may need to add these lines again to change the color of the payload. Otherwise, it will be white.

## Validation and Specification Links
* See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

* Sensor specification links:
  * Velodyne-16 https://www.mapix.com/wp-content/uploads/2018/07/63-9229_Rev-H_Puck-_Datasheet_Web-1.pdf
  * Depth Camera https://www.intelrealsense.com/depth-camera-d435
  * IMU: XSens MTI-100 https://www.mouser.com/datasheet/2/693/mti_100_series-1540263.pdf
