# CTU\_CRAS\_NORLAB\_TRAILER\_SENSOR\_CONFIG\_1

This package contains sensor config 1 of CTU-CRAS-Norlab comms extension trailer.
It is a passive robot that is tethered to the teambase via wired Ethernet. It is
expected to be dragged by a UGV as a marsupial.

The cable length is 100 m. When distance between the trailer and teambase is
longer, they can only communicate via standard SubT comms model. If they are
close enough, their communication has unlimited bandwidth and they are treated
as if they were in the very same place.

For the "wire-like" behavior, this robot's name has to be COMMS_EXTENDER and
teambase's name has to be TEAMBASE.

## Testing the communication

To get better idea what's happening, open `subt_communication_broker/src/subt_communication_broker.cpp`,
find function `DispatchMessages()` and scroll to a line reading `std::tie(sendPacket, rssi) =`.
A little above it you'll find the logic that decides whether the communication should go
"over the wire". And there are two debug printouts which might come handy when testing this
feature. So uncomment them and rebuild your workspace.

This package provides a program called `test_comms` that can be used to create a high amount of
SubT comms traffic, on which it will be apparent that the comms going "over the wire" helps the
resulting speed of transfer.

Start simulator:

    LANG=C ign launch -v4 competition.ign robotName1:=TEAMBASE robotConfig1:=TEAMBASE robotName2:=COMMS_EXTENDER robotConfig2:=X1_SENSOR_CONFIG_1 circuit:=cave worldName:=simple_cave_01

Then open another terminal and start 

    ROS_NAMESPACE=COMMS_EXTENDER ./test_comms

And yet another terminal and start 

    ROS_NAMESPACE=TEAMBASE ./test_comms _robot_name:=TEAMBASE _dst_name:=COMMS_EXTENDER

Now you should see the nodes printing about statistics about the transferred data. And the terminal
with simulator should show you the debug printouts. If you did not rebuild it with the debug
printouts, you can still tell that the comms is going over the wire from the VisibilityRfTable debug
printouts, where you'll see `Range: 0`. Now move the comms extender somewhere and watch for the
distance. As soon as it gets 100 m from the teambase, comms start dropping in both signal quality and
delivery success ratio.

## Model files management

This package follows a different workflow for managing SDF and URDF models of the robot than the suggested one. The only and main source of model data is `urdf/trailer.xacro` file and the files it includes. 

To get the URDF model of the robot, call `scripts/print_robot_urdf` script which prints the robot URDF on stdout. This script is used in `launch/description.launch`.

The SDF model is a regular file committed to this package, but its updates are not
to be done manually. To change the robot model, make changes in the Xacro, and then run script `scripts/update_robot_sdf`, which updates the `model.sdf` file in this repo. The change can then be commited.