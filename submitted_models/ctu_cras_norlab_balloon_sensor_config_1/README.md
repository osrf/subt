# CTU\_CRAS\_NORLAB\_BALLOON\_SENSOR\_CONFIG\_1

This package contains sensor config 1 of CTU-CRAS-Norlab Helium balloon.
It is a passive robot that is tethered to a UGV via a reel and has one wide-angle
fisheye camera, 3 lights, battery and a wifi-enabled micro board that is able to send the
images to the UGV.

The tether length is 15 m.

This robot requires the Buoyancy world plugin for it to actually float. 

## Model files management

This package follows a different workflow for managing SDF and URDF models of the robot than the suggested one. The only and main source of model data is `urdf/trailer.xacro` file and the files it includes. 

To get the URDF model of the robot, call `scripts/print_robot_urdf` script which prints the robot URDF on stdout. This script is used in `launch/description.launch`.

The SDF model is a regular file committed to this package, but its updates are not
to be done manually. To change the robot model, make changes in the Xacro, and then run script `scripts/update_robot_sdf`, which updates the `model.sdf` file in this repo. The change can then be commited.