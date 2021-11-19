# README #

This README would normally document whatever steps are necessary to get your application up and running.

## What is this repository for?

CMU SubT Explorer Canary1 model in SubT-Ignition Simulation Enviroment. 

## How to get the ignition runing on your workspace 

You may follow the instruction here to get the simulation running in your workspace.
[tutorials](https://github.com/osrf/subt/wiki/Tutorials)

## How to spawn this model in ignition 

#### Copy this repo into your workspace
```bash
cd <<SubT Ignition workspace>>/src/subt/submitted_models
git clone git@bitbucket.org:cmusubt/explorer_canary1_sensor_config_1.git
cd ../../
catkin_make install
source install/setup.bash
```

#### Run Ignition
```bash
ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X3 robotConfig1:=EXPLORER_CANARY1_SENSOR_CONFIG_1 localModel:=true enableGroundTruth:=true
```

## How to control the drone 
1. Publish message to topic "/X3/velocity_controller/enable" to set it to true
2. Publish velocity control command to "/X3/cmd_vel"
3. You may directly use the default joystick controll node provided by DARPA, kindly follow the instruction here. [instruction](https://bitbucket.org/osrf/subt/wiki/tutorials/ExampleSetup)

## How to run state estimation - [Lego-LOAM]

#### Download repository ``` lego_loam_velodyne16 ``` into your workspace, and follow the instruction in README.md to install any dependences.
```bash
cd <<SubT workspace>>/src
git clone git@bitbucket.org:cmusubt/lego_loam_velodyne16.git
catkin build
```
#### Complile Failure
You may face issue compiling the package at the first time. There is a confilt of the system LZ4 library in Ubuntu 18.04 with ROS melodic, a temporary fix to resolve this. [reference](https://github.com/ethz-asl/lidar_align/issues/16)

``` bash
vim /usr/include/flann/util/serialization.h
```
Change the corresponding lines of include libraries:
```
#inclue flann/lz4.h ->  #include lz4.h
#inclue flann/lz4hc.h ->  #include lz4hc.h
```

#### Run Lego-LOAM
```bash
roslaunch lego_loam_bor run.launch robot_name:=X3

```

#### Activate Ground Truth Odometry
```bash
roslaunch lego_loam_bor run.launch robot_name:=X3 use_ground_truth:=true

```




## What sensors it has 
1. 3D medium range lidar (Velodyne)
2. 3D medium range lidar (Ouster)
3. IMU
4. Gas sensor
5. Pressure sensor
6. Magnetometer
7. 2 fisheye cameras (simulated by 4 cameras since the are no fisheye models in the sim)


## Who do I talk to? 

John Keller
(jkeller2@andrew.cmu.edu)
