# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

CMU SubT Explorer DS1 model in SubT-Ignition Simulation Enviroment. 

### How to get the ignition runing on your workspace ###

You may follow the instruction here to get the simulation running in your workspace.
[tutorials](https://bitbucket.org/osrf/subt/wiki/documentation)

### How to spawn this model in ignition ###

1. Copy this repo into your workspace
```bash
cd <<SubT Ignition workspace>>/src/subt/submitted_models
git clone git@bitbucket.org:cmusubt/explorer_ds1_sensor_config_1.git
cd ../../
catkin_make install
source install/setup.bash
```

2. Run Ignition
```bash
ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=ds1 robotConfig1:=EXPLORER_DS1_SENSOR_CONFIG_1 localModel:=true
```
### How to control the drone ###
1. Publish message to topic "ds1/velocity_controller/enable" to set it to true
2. Publish velocity control command to "ds1/cmd_vel"


### What sensor it has ###
1. 3D medium range lidar, 
2. IMU, 
3. Gas sensor,
4. Pressure sensor, 
5. Magnetometer, 
6. Three Realsense Camera [Up, Down, Front]


### Who do I talk to? ###

Fan Yang
(fanyang2@andrew.cmu.edu)
