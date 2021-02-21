# ARIAC 2021 Workspace

## Quick usage

In the host system, clone the repo:

> git clone --recursive git@github.com:glpuga/ariac2021ws.git

> cd ariac2021ws

> git checkout devel

Build the container:

> cd docker

> ./build

> ./run

In the container, build the worspace and launch the sample environment:

> catkin_make

> source devel/setup.bash

> roslaunch nist_gear sample_environment.launch

**Note:** To step around the long load time mentioned in [Issue #3](https://github.com/glpuga/ariac2021ws/issues/3), use the alternative launch file:

> roslaunch devutils sample_environment.launch

## Links and documents

ARIAC 2021:

* [ARIAC main site](https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition).
* [ARIAC code repository](https://github.com/usnistgov/ARIAC/tree/ariac2021).
* [ARIAC competition server code](https://github.com/zeidk/ariac-docker). This might be old, since it hasn't been updated in a couple of years.


## Important Dates

* Qualifiers: April/26 - April/30
* Finals: May/17 - May/28

## Usage

### Setting up the local repository

Notice that this repository uses submodules. To clone the repository use:

> git clone --recursive git@github.com:glpuga/ariac2021ws.git

> cd ariac2021ws

> git checkout devel

### Development container

Within the `docker` folder there are files to build and run a docker container with a preinstalled ROS Melodic / Ubuntu 18.04 environment able to run the competition environment.

The docker file can be built, run and joined using the helper scripts present in the folder. The `build.sh` script will create the docker image and leave it ready to run.

> ./build.sh

To launch the first instance of the container, use the `run.sh` script, with no parameters.

> ./run.sh

If a second terminal into the same running container is needed, use the `join.sh` container. 

> ./join.sh

Notice that for simple terminal multiplexing, `tmux` has been installed within the container.

### Development environment

Once within the development container, the work folder will be `/home/developer/ws/`, with the current repo mounted under the `src/` folder.

Also, the default ROS environment is preloaded by the container entry configuration.

To build the workspace just call `catkin_make`.

> catkin_make

> source devel/setup.bash

### Launching the sample environment

There's a sample environment within the `nist_gear` package in the ARIAC repository. To launch, run the container, build and launch as follows:

> roslaunch nist_gear sample_environment.launch

### Controlling the gantry/kitting using RViz

It's possible to dos some basic control of the robot arms using RViz.

**Note:** Note that this environment is affeced by a bug, detailed in [Issue #3](https://github.com/glpuga/ariac2021ws/issues/3), which causes the joint controllers to crash and the arms to be come not responsive to planning and control. The following instructions are therefore _how it should be done_ if not for the bug. Keep reading and you'll find below instructions to step around the issue while it gets addressed upstream.

Start by launching the sample environment with **moveit**, using the following launch file:

> roslaunch nist_gear sample_environment.launch load_moveit:=true

then in a separate terminal launch either

> roslaunch devutils rviz_control_gantry.launch

to control the gantry, or for the manipulator in the kitting launch:

> roslaunch devutils rviz_control_kitting.launch

#### To overcome Issue #3

To step around the problem described in [Issue #3](https://github.com/glpuga/ariac2021ws/issues/3), replace

> roslaunch nist_gear sample_environment.launch load_moveit:=true

in the previous instructions with this alternative launch file:

> roslaunch devutils devutils sample_environment.launch

Notice that the alternative launch file does not require the `load_moveit` parameter, since it defaults to `true`, unlike the original launch file which defaults to false.

This should result in a much speedier load time, and the possiblity to plan and move the manipulators using RViz.
