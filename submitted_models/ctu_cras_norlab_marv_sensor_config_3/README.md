# CTU\_CRAS\_NORLAB\_MARV\_SENSOR\_CONFIG\_3

This package contains sensor config 3 of CTU-CRAS-Norlab MARV robot (Mobile Autonomous Rescue Vehicle). The robot has:

- 5 Basler Ace 2 Pro a2A1920-51gcPRO RGB cameras with 86° FOV lenses (Basler C125-0418-5M-P f4mm)
- 2 Basler Ace acA2040-35gc RGB cameras with 138° FOV fisheye lenses (Evetar M118B029520IR)
- Ouster OS0-128 lidar
- 4 TFMini plus point lidars
- IMU Xsens MTI-30
- Gas sensor
- Thermal camera FLIR Boson 92° FOV
- Weight: about 61 kg

## Model files management

This package follows a different workflow for managing SDF and URDF models of the robot than the suggested one. The only and main source of model data is `urdf/marv.xacro` file in [`ctu_cras_norlab_marv_sensor_config_1`](../ctu_cras_norlab_marv_sensor_config_1) package and the files it includes.

To get the URDF model of the robot, call `scripts/print_robot_urdf` script which prints the robot URDF on stdout. This script is used in `launch/description.launch`.

The SDF model is a regular file committed to this package, but its updates are not
to be done manually. To change the robot model, make changes in the Xacro, and then run script `scripts/update_robot_sdf_ign`, which updates the `model.sdf` file in this repo. The change can then be commited.

There is also script `scripts/update_robot_sdf_gz`, which creates a model.sdf file suitable for use in Gazebo Classic. This can be used to get its much better visualization functionality, i.e. viewing transparent model, wireframe, inertia, centers of gravity etc.

The model is also configured by a set of shared config files located in `config/` directory of this package and sensor config 1. It contains YAML files which are loaded in the following order:

- URDF: `sensor_config_1/config/common.yaml`, `sensor_config_1/config/urdf.yaml`, `common.yaml`, `urdf.yaml`
- SDF for Ignition Gazebo: `sensor_config_1/config/common.yaml`, `sensor_config_1/config/sim.yaml`, `sensor_config_1/config/ign.yaml`, `common.yaml`, `sim.yaml`, `ign.yaml`
- SDF for Gazebo Classic: `sensor_config_1/config/common.yaml`, `sensor_config_1/config/sim.yaml`, `sensor_config_1/config/gz.yaml`, `common.yaml`, `sim.yaml`, `gz.yaml`