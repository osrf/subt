# Boston Dynamics Spot - CTU-CRAS-Norlab sensor config
This specifications.md file is a description and proof of virtual model validation for the
Boston Dynamics Spot robot configured by team CTU-CRAS-Norlab. This robot may be launched using an
ign launch command with the variable name `CTU_CRAS_NORLAB_SPOT_SENSOR_CONFIG_1`.

For local testing, you can use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/ctu_cras_norlab_spot_sensor_config_1/launch/example.ign robotName:=X1 ros:=true champ:=true

This specifications.md file describes mainly the additional sensory payload of our Spot. Refer to
[bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md) for information about the robot base,
how to control it and how to modify parameters of the robot.

## Description
This sensor config carries 5 FullHD RGB cameras with global shutter, an IMU and a 3D lidar.
Sensor config 2 has additional 12 comms breadcrumbs.

## Usage Instructions

See [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md). There is no diversion from
the principles described there, just substitute `bosdyn_spot` with `ctu_cras_norlab_spot_sensor_config_1`.

## Usage Rights
No additional restrictions have to be taken into account for this model.

## Cost and Scale
* Base vehicle: $75,000
* Custom omnicamera solution: $5,000
* IMU: $1,500
* Computers: $1,500
* Lidar: $10,000
* Total price: $93,000

Its weight is approximately 38.66 kg (or 39.16 kg with breadcrumbs). 

## Sensors
This sensor configuration of Spot includes the following sensors in addition to the base sensors of the platform
described in [bosdyn_spot/specifications.md](../bosdyn_spot/specifications.md).
The specifications for these instruments are provided below in the [Validation Links](#validation_links) section.

The following specific sensors are declared payloads of this vehicle:

* Ouster OS0-128 3D lidar modeled by `gpu_lidar` sensor. It runs configured to produce 2048x128 scans at 10 Hz. It has a very wide vertical field of view (90 degrees) and short range (about 50 meters).
* 5x RGB global shutter camera Basler a2A1920-51gcPRO with 4 mm lens modeled by `camera` sensor. Each camera provides 86 degrees of horizontal FOV. Resolution is 1920x1200 px. According to datasheet, the camera can run on frequencies up to 51 Hz, but the robot doesn't have sufficient computational and communication capacity to transmit such high frequency data. We thus lowered the framerate of the simulated camera to 9 Hz. We verified in real life that we are able to transmit the 9 Hz images and process them (the 5 cameras more or less saturate a Gigabit link with this framerate).
* XSens MTI-30 IMU: modeled by `imu` sensor.
* 12 communication breadcrumbs are also available as a payload for this robot in sensor configuration 2 (real robot will carry 4 or 8 breadcrumbs).

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
  * Ouster OS0-128 https://ouster.com/products/os0-lidar-sensor/
  * Basler cam https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace2/a2a1920-51gcpro/ + lens https://www.baslerweb.com/en/products/vision-components/lenses/basler-lens-c125-0418-5m-p-f4mm/ + Experimental validation: https://drive.google.com/drive/folders/1-KWMvjWSYYeLqPA0C3FCdKuHQAmOHN-z?usp=sharing
  * IMU: XSens MTi-30 https://www.mouser.com/datasheet/2/693/mti-series-1358510.pdf
  * Lights: the robot uses LED strips around the body. The total power output of these strips is about `30 W`. It also has a ceiling-pointing light.