# Playback Event Recorder

The playback event recorder creates videos for events that are stored in the
events.yml log file in the state log directory.

Here are the events being recorded and their associated camera mode:

* robots exiting staging area: static camera above staging area looking down at angle
* rock fall: static camera above dynamic rocks model looking directly downward
* marsupial vehicle detaching child robot: camera follows the child robot
* artifact proximity: camera follows the robot
* region of interest (decision, vertical, elevation): camera follows the robot

## Running the demo

To create event videos for a subt log, run:

```
bash record_playback_events.bash <path_to_log_dir>
```

The script launches ign gazebo in playback mode and creates videos using the
GUI camera video recording feature. For each event, it seeks playback to some
time before the event, moves the camera to the desired location (either in
follow mode or moves to a static pose), and starts recording until some time
after the event. After recording all events, ign-gazebo will exit on its own
and you can find the recorded videos in a timestamped directory inside the
directory where the bash script is run.

## Video recorder settings

By default, the video recorder plugin records videos based on real time and
in lower quality than desired. In order to produce smooth high quality videos
with accurate timing, we need to specify a few video recorder
configurations in the `gui.config` file. The config file should be located in
`$HOME/.ignition/gazebo`. Find the 3D Scene plugin and add the following
`<record_video>` settings:

```xml
<!-- 3D scene -->
<plugin filename="GzScene3D" name="3D View">
  <ignition-gui>
    <title>3D View</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
  </ignition-gui>

  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>6 0 6 0 0.5 3.14</camera_pose>

  <record_video>
    <use_sim_time>true</use_sim_time>
    <lockstep>true</lockstep>
    <bitrate>8000000</bitrate>
  </record_video>

</plugin>
```

This makes sure the video recording is done in lock step with simulation
state updates so that we do not miss any frames during encoding. Additionally
sim time is used as timestamp so that the generated video length is unaffected
by the speed at which log playback is running in. For example, it may take
up to several hours to playback an event in a subt log due to low RTF but the
resulting video should still have a length that matches the specified duration
of the event.

## Known issue

The video recorder configurations are set to produce higher quality videos
but the encoding process is also more expensive. Since the server and the GUI
run asynchronously in two separate processes, we usually see that the GUI start
to lag behind the server after several minutes into recording. Once the ignition
transport msg queue is full, it starts dropping state msgs, and we would
notice missing frames in the generated videos. This is currently mitigated by
using a "catch-up" recording strategy in the playback event recorder, which
basically pauses the simulation when we detect the the video recorder is lagging
behind the server above some threshold, and only resumes playing back simulation
when the lag is within a reasonable amount of time.
