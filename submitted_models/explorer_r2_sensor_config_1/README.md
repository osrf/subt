# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

CMU SubT Explorer R2 model in SubT-Ignition Simulation Enviroment. 

### How to get the ignition runing on your workspace ###

You may follow the instruction here to get the simulation running in your workspace.
[tutorials](https://github.com/osrf/subt/wiki/Tutorials)

### How to spawn this model in ignition ###

1. Copy this repo into your workspace
```bash
cd <<SubT Ignition workspace>>/src/subt/submitted_models
git clone git@bitbucket.org:cmusubt/explorer_r2_sensor_config_1.git
cd ../../
catkin_make install
source install/setup.bash
```

2. Run Ignition
```bash
ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=r2 robotConfig1:=EXPLORER_R2_SENSOR_CONFIG_1 localModel:=true
```
### How to control the drone ###
1. Publish velocity control command to "r2/cmd_vel"


### how to get odometry ###
1.git clone git@bitbucket.org:cmusubt/payload_sim.git
2.change "<param name="simulationSelect" type="string" value="gazebo" />" to "<param name="simulationSelect" type="string" value="ignition" />"
3.roslaunch payload_sim payload_sim.lanch


### Who do I talk to? ###

Hongbiao Zhu
(hongbiaz@andrew.cmu.edu)
