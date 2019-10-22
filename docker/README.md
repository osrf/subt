# Overview

This directory contains scripts and Dockerfiles for use during development
and testing of SubT Solutions.

## Dockerfile Types

Each subdirectory contains a Dockerfile for a particular use case.

### `cloudsim_sim`

This is the docker image that will run a simulation instance in the
same way as CloudSim. Refer to: https://github.com/osrf/subt/wiki/Cloudsim%20Architecture.

This image should be used in conjunction with `cloudsim_bridge`.

**Usage**

1. Build the docker image

```
./build.bash cloudsim_sim
```

2. Run the docker image.

```
./run.bash cloudsim_sim cloudsim_sim.ign robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1  robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG2
```

### `cloudsim_bridge`

This is the docker image that with run the cloudsim bridge. Refer to:
https://github.com/osrf/subt/wiki/Cloudsim%20Architecture.

This image should be used in conjunction with `cloudsim_sim`.

**Usage**

1. Build the docker image

```
./build.bash cloudsim_bridge
```

2. Run the docker image.

```
./run.bash cloudsim_bridge robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1 robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG2
```

### `subt_shell`

Use this directory to build a Docker image that supports an interactive bash shell and contains the SubT code base.

**Usage**

1. Build the docker image

```
./build.bash subt_shell
```

2. Run the docker image.

```
./run.bash subt_shell
```

3. Join a running docker image

```
./join.bash subt_shell

```

### `subt_sim_entry`

Use this directory to build a Docker image that will run an installed launch file.

**Usage**

1. Build the docker image

```
./build.bash subt_sim_entry
```

2. Run the docker image. The last command line option is the name of launch
   file.

```
./run.bash subt_sim_entry tunne_circuit_practice.ign
```
