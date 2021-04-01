# CTU-CRAS-Norlab Balloon
This specifications.md file is a description and proof of virtual model validation for the CTU-CRAS-Norlab helium balloon. This model may be launched using an ign launch command with the variable name `CTU_CRAS_NORLAB_BALLOON_SENSOR_CONFIG_1`.

For local testing, you can also use the following command:

    LC_ALL=C ign launch -v 4 ~/subt_ws/src/subt/submitted_models/ctu_cras_norlab_balloon_sensor_config_1/launch/example.ign robotName:=X1 ros:=true

## Description
This package contains sensor config 1 of CTU-CRAS-Norlab Helium balloon.
It is a passive robot that is tethered to a UGV via a reel and has one wide-angle
fisheye camera, 3 lights, battery and a wifi-enabled micro board that is able to send the
images to the UGV.

The tether length is 15 m.

This robot requires the Buoyancy world plugin for it to actually float.

This robot is only expected to be used as a marsupial child, not standalone. 

The model is connected to the UGVs as other marsupials, but this one should never be detached!

## Usage Instructions

The balloon is tethered on a reel that is controlled by publishing velocity commands 
of type `std_msgs/Float64` to topic `reel_cmd_vel`. The commands can go from -0.5 to 0.5
and specify the unreeling of the tether. As the balloon floats, it automatically raises
when the thether is extended.

Positive commands mean extension of the reel, negative mean pulling the balloon closer
to the UGV. Zero means no change. 

It is okay to publish -0.5 all the time except the instants when you actually want
the balloon to go exploring.

## Usage Rights
No additional restrictions have to be taken into account for this configuration.

## Cost and Scale
This is a custom-built vehicle.

* Base vehicle: $1 
* Camera: $35
* Fisheye lens: $20
* Battery: $20
* Other electronics: $10
* The reeling mechanism: $50
* Total: ~ $136

The weight of the vehicle is about 50 grams (excluding the reel). The reel has not yet been
constructed, so we will add details as soon as we have it.

## Sensors

The balloon is equipped with one fisheye RGB camera. As Ignition Gazebo cannot simulate fisheye
lenses with very wide field of view, we substituted this camera with 5 90° cameras with comparable
sum of resolutions.

Camera: Arducam mini 2Mpx OV2640 SPI camera shield (22 grams). RGB camera with resolution 1600x1200 px.
Lens: Arducam 1/4" 1,05mm f/2,0 M12 lens M40105M19, providing 194° horizontal FOV with fisheye projection.

The single camera is simulated by 5 `camera` sensors with resolution 800x600 px and each with horizontal
FOV of 90°.

## Control

The balloon is tethered on a reel that is controlled by publishing velocity commands
of type `std_msgs/Float64` to topic `reel_cmd_vel`. The commands can go from -0.5 to 0.5
and specify the unreeling of the tether. As the balloon floats, it automatically raises
when the thether is extended.

Positive commands mean extension of the reel, negative mean pulling the balloon closer
to the UGV. Zero means no change.

It is okay to publish -0.5 all the time except the instants when you actually want
the balloon to go exploring.

The transformation between the balloon and the reel (`X1`->`X1/balloon`) is explicitly removed from the ROS TF tree. The
real balloon has no means of localization and has to rely on visual localization using an Apriltag
that is mounted on the top of the reel mechanism. It is expected that this localization will be done
by the UGV and not by the balloon as it has almost no computational capabilites onboard.

## Motion characteristics

The balloon floats in opposite direction to gravity and is tied to the marsupial parent via tether.
The tehter is simulated by a frictionless prismatic joint with stops at 0 and 15 meters. The reel
can control the position of this prismatic joint by issuing velocity commands.

We did a test with this balloon in real caves in 2020 and we empirically measured that it can lift
at least 30 g payload and keep floating for much more than 1 hour.

## Endurance Characteristics

This vehicle has not yet been built in whole, so we don't yet know the final endurance. But we aim
for 60 minutes.

## Diversions from Physical Hardware

The tether is not visualized in any way.

The simulated tether cannot collide with the environment and cannot bend or tie up.

It is actually possible to "push" the balloon by issuing positive velocity commands. That would not
work in the real world. 

The 194° fisheye camera is simulated by 5 90° cameras.

The real balloon has aluminium surface so it is much more reflective.

The weights of parts of the simulated model do not correspond to reality (they might be up to
2x higher than the real ones). But these are the numbers that provide solid a stable simulation. If you
ever see the simulator crashing with NaNs going in and out of ODE, the simulation destabilized too much.
I saw it a lot when setting up the model, but adding internal friction to the ball joint helped this
and I haven't seen a crash since correcting the parameters.

## Changing parameters of the robot

Look into the Xacro files in `urdf/` folder and change the values there.

After changing a value in these files, run script `scripts/update_robot_sdf` to write the changes to the SDF file.

__Do not change the SDF files directly, as the changes could be lost the next time the update scripts are run.__

## Validation and Specification Links
* Vehicle Links:
  * Base vehicle: https://www.balonky.cz/obchod/balonky-3d/foliovy-balonek-stribrna-koule-velka-53-cm-x-53-cm-3910199
  
* We think this vehicle does not require validation. But we can provide a video of the
  balloon going up with a payload of approx. 30 g.

* Specification links:
  * Arducam mini 2Mpx OV2640 SPI camera shield https://rpishop.cz/kamery/2803-arducam-mini-2mpx-ov2640-spi-kamera-shield.html
  * Arducam 1/4" 1,05mm f/2,0 M12 lens M40105M19 https://rpishop.cz/objektivy/2923-arducam-14-105mm-f20-m12-objektiv-m40105m19.html