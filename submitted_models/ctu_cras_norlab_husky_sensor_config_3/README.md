# CTU\_CRAS\_NORLAB\_HUSKY\_SENSOR\_CONFIG\_3

This package contains sensor config 3 of CTU-CRAS-Norlab Husky robot. The robot has:

- Ouster OS0-128 lidar
- An omnivision setup of 4 Basler Ace2 (a2A1920-51gcPRO) cameras with C125-0418-5M-P Basler Lens
- LED strips
- Xsens MTi-30 IMU
- Weight: about 58 kg

## Model files management

This package follows a different workflow for managing SDF and URDF models of the robot than the suggested one. The only and main source of model data is `urdf/robot.xacro` file and the files it includes. 

To get the URDF model of the robot, call `scripts/print_robot_urdf` script which prints the robot URDF on stdout. This script is used in `launch/description.launch`.

The SDF model is a regular file committed to this package, but its updates are not
to be done manually. To change the robot model, make changes in the Xacro, and then run script `scripts/update_robot_sdf_ign`, which updates the `model.sdf` file in this repo. The change can then be commited.

There is also script `scripts/update_robot_sdf_gz`, which creates a model.sdf file suitable for use in Gazebo Classic. This can be used to get its much better visualization functionality, i.e. viewing transparent model, wireframe, inertia, centers of gravity etc.

The model is also configured by a set of shared config files located in `config/` directory. It contains YAML files which are loaded in the following order:

- URDF: `common.yaml`, `urdf.yaml`
- SDF for Ignition Gazebo: `common.yaml`, `sim.yaml`, `ign.yaml`
- SDF for Gazebo Classic: `common.yaml`, `sim.yaml`, `gz.yaml`

Sensor configuration 4 of Husky makes use of the robot model defined here, but uses customized configuration files to get the right sensor suite.