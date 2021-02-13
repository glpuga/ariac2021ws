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

## Links and documents

ARIAC 2021:

* [ARIAC main site](https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition).
* [ARIAC code repository](https://github.com/usnistgov/ARIAC/tree/ariac2021).


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

