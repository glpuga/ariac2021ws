![Version](https://img.shields.io/badge/version-1.0-blue.svg)
![Industrial CI](https://github.com/glpuga/ariac2021ws/actions/workflows/industrial_ci.yaml/badge.svg?branch=devel)
![Devel Container](https://github.com/glpuga/ariac2021ws/actions/workflows/devel_docker_build_and_test.yaml/badge.svg?branch=devel)

# ARIAC 2022 TheItalianJob workspace

## ARIAC 2022

Links and documents:

* [ARIAC main site](https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition).
* [ARIAC code repository](https://github.com/usnistgov/ARIAC).
* [ARIAC competition server code](https://github.com/osrf/ariac-docker).
* [Project](https://github.com/glpuga/ariac2021ws/projects/1)


## Quick usage

In the host system, clone the repo:

```
git clone --recursive git@github.com:glpuga/ariac2021ws.git
cd ariac2021ws
git checkout main
```

Build the container:

```
cd docker
./build
./run.sh
```

Within the container build the worspace and launch the sample environment:

```
catkin_make
source devel/setup.bash
roslaunch tijchallenger gear.launch trial_config:=sample_kitting.yaml
```

That will launch the simulated environment. It may take a bit to load the first time since Gazebo will download a number of models to prepare the environment.

Once the simulation is up, open a second terminal and join the same running container as before:

```
cd ariac2021ws/docker
./join.sh
source devel/setup.bash
```

Finally, launch the competition node.

```
roslaunch tijchallenger challenger.launch
```

The node will start the competition mode and the robots will begin completing the orders in the trial configuration file.

