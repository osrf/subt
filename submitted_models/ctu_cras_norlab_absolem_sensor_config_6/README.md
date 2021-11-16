# CTU\_CRAS\_NORLAB\_ABSOLEM\_SENSOR\_CONFIG\_6

This package contains sensor config 6 of CTU-CRAS-Norlab Absolem robot. The robot has:

- 5x Basler ACE Pro 2 RGB camera
- Ouster OS0-128 lidar
- 4 TFMini plus point lidars
- IMU Xsens MTI-30
- Gas sensor
- Weight: about 38 kg
- It is 50% faster than sensor configs 1 and 2 due to a fixed mechanical issue in the tracks

## Model files management

This package follows a different workflow for managing SDF and URDF models of the robot than the suggested one. The only and main source of model data is `urdf/nifti_robot.xacro` file in [`ctu_cras_norlab_absolem_sensor_config_1`](../ctu_cras_norlab_absolem_sensor_config_1) package and the files it includes.

To get the URDF model of the robot, call `scripts/print_robot_urdf` script which prints the robot URDF on stdout. This script is used in `launch/description.launch`.

The SDF model is a regular file committed to this package, but its updates are not
to be done manually. To change the robot model, make changes in the Xacro, and then run script `scripts/update_robot_sdf`, which updates the `model.sdf` file in this repo. The change can then be commited.

The model is also configured by a set of shared config files located in `config/` directory of this package and sensor config 1. It contains YAML files which are loaded in the following order:

- URDF: `sensor_config_1/config/common.yaml`, `sensor_config_1/config/urdf.yaml`, `sensor_config_3/config/common.yaml`, `sensor_config_3/config/urdf.yaml`, `common.yaml`, `urdf.yaml`
- SDF for Ignition Gazebo: `sensor_config_1/config/common.yaml`, `sensor_config_1/config/sdf.yaml`, `sensor_config_3/config/common.yaml`, `sensor_config_3/config/sdf.yaml`, `common.yaml`, `sdf.yaml`