# Boston Dynamics Spot - Marble sensor config 2
This specifications.md file is a description and proof of virtual model validation for the
Boston Dynamics Spot robot configured by team Marble. This robot may be launched using an
ign launch command with the variable name `marble_spot_sensor_config_2`.

For local testing, you can use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/marble_spot_sensor_config_2/launch/example.ign robotName:=X1 ros:=true champ:=true

This specifications.md file describes mainly the additional sensory payload of our Spot. Refer to
[bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md) for information about the robot base,
how to control it and how to modify parameters of the robot.

## Description
This sensor config carries 3 wide-angle RGB cameras with global shutter, an IMU and a 3D lidar.

## Usage Instructions

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md). There is no diversion from
the principles described there, just substitute `bosdyn_spot` with `marble_spot_sensor_config_2`.

## Usage Rights
No additional restrictions have to be taken into account for this model.

## Cost and Scale
* Base vehicle: $75,000
* Flir Cameras + Lenses: $2,500
* IMU: $1,500
* Computers: $1,500
* Lidar: $6,000
* Total price: $88,500

vehicle weight: TODO

## Sensors
This sensor configuration of Spot includes the following sensors in addition to the base sensors of the platform
described in [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Ouster OS1-Gen1 3D lidar modeled by `gpu_lidar` sensor. It runs configured to produce 1024x64 scans at 10 Hz. It has a vertical field of view (33 degrees) and 120 meter range
* 3x RGB global shutter FLIR Backfly cameras modeled by `camera` sensor. Each camera provides 80 degrees of horizontal FOV. Resolution is 808x608 px. According to datasheet, the camera can run on frequencies up to 51 Hz, but the robot doesn't have sufficient computational and communication capacity to transmit such high frequency data. We thus lowered the framerate of the simulated camera to 10 Hz. 
* XSens MTI-30 IMU: modeled by `imu` sensor.

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

The Ouster lidar produces the 360 degree scans in a single instant in simulation, as opposed to the 100 ms-long interval it takes the real lidar.

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

## Validation and Specification Links
* See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).

* Sensor specification links:
  * Ouster OS1-64 https://data.ouster.io/downloads/datasheets/datasheet-gen1-v1p13-os1.pdf
  * FLIR Backfly Camera https://flir.app.boxcn.net/s/iicqenjhtth41dt13951qh5toidx29ih + lens https://www.bhphotovideo.com/c/product/414239-REG/Tamron_13FM28IR_13FM28IR_2_8_mm_f_1_2.html
  * IMU: XSens MTi-30 https://www.mouser.com/datasheet/2/693/mti-series-1358510.pdf
  * Lights: the robot uses LED strips around the body. The total power output of these strips is about `30 W`. It also has a ceiling-pointing light.
