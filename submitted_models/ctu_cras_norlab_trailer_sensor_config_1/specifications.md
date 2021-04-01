# CTU-CRAS-Norlab Trailer
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-Norlab trailer. This model may be launched using an ign launch command with the variable name `CTU_CRAS_NORLAB_TRAILER_SENSOR_CONFIG_1`.

For local testing, you can also use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/ctu_cras_norlab_trailer_sensor_config_1/launch/example.ign robotName:=X1

## Description
This is a 2-wheel trailer which carries a large communication node Mobilicom MCU-200.
The trailer is connected to the teambase via a 100-meter 2-wire cable.
The usage of 2N 2-wire converters allows to run 100 Mbps Ethernet on this
very thin and easily reel-able cable. It is expected that in the teambase there is a team member that
oversights the unreeling of the cable and helps the cable unreel if necessary.

The trailer attaches behind one of the ground robots via the same mechanism marsupial UAVs do. It can
be detached any time.

## Usage Instructions
The robot is completely passive (except for its ability to break the marsupial joint).
It acts as a normal subt comms network participant.

However, if it is up to 100 m from teambase, it uses a "direct comms link" that goes around
the standard limitations of subt communication. There is no theoretical bitrate limit for 
bidirectional communication between the teambase and the trailer. The data they exchange
do not count in their comms budget with other teammates. Packet drop rate corresponds to
communication via the normal comms model with zero distance.

For the direct comms to work, teambase has to have name `TEAMBASE` and this trailer
has to have name `COMMS_EXTENDER`. With other names, the direct comms link will not
be used.

See also [README.md]() of this package for details on how to locally test the direct comms.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

## Cost and Scale
This is a custom-built vehicle.

* Base vehicle: $100 
* Large communication node Mobilicom MCU-200: $10,000
* Total: ~ $10,100

The weight of the vehicle including the communication node and enough batteries is about 5-6 kg.

## Sensors

No sensors.

## Control

No control. This is a passive vehicle.

## Motion characteristics

The vehicle is intended to be dragged behind a UGV. It's motion will comply to the mechanics of these
two connected bodies. The dragging rod is fixed to the trailer body, and its loose end contains a
2 DOF joint via which the trailer attaches to the UGV. This joint is also completely passive (but it
can be broken by the detach command). 

## Endurance Characteristics
Since there is no big pressure on weight of this vehicle and the only power-consuming element is the
comms node with up to 50 W consumption (20 W average), it can be loaded with as many batteries as needed
and thus its endurance goes far beyond the scope of the SubT challenge. We thus suggest assigning it
endurance of 90 minutes, or no power limit at all.

## Diversions from Physical Hardware

The dragging rod can't collide with the marsupiual partner in simulation.

The actual working of the comms node is substituted with the SubT comms broker.

The real trailer is also connected to the teambase via the physical wire which is
not modeled. So the real trailer could not go further than 100 m from teambase.
The simulated trailer can go anywhere, but further than 100 m from teambase it loses
its extra communication capability, making it essentially a standard breadcrumb.

The real detachment mechanism will be controlled by the robot as the trailer has no compute.

The real breakable joint will have some force limit above which it will safely break
even without a command. The simulated joint is unbreakable until commanded.

## Changing parameters of the robot

Look into the Xacro files in `urdf/` folder and change the values there.

After changing a value in these files, run script `scripts/update_robot_sdf` to write the changes to the SDF file.

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

## Validation and Specification Links
* Vehicle Links:
  * custom-made vehicle using 16" kickbike wheels
  
* We think this vehicle does not require validation. But we can provide a video of the
  communication reaching 90 Mbps over a 100 m reel of 2-wire cable.

* Specification links:
  * 2N 2-wire converter https://www.2n.cz/en_GB/products/intercoms/2n-2wire
  * Mobilicom MCU-200 comms node https://www.mobilicom.com/mcu-200-ruggedized