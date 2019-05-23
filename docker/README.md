# Overview

This directory contains scripts and Dockerfiles for use during development
and testing of SubT Solutions.

## Dockerfile Types

Each subdirectory contains a Dockerfile for a particular use case.

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
./run.bash subt_sim_entry virtual_stix.ign
```
