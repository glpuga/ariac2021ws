#!/usr/bin/env bash

source /opt/ros/${ROS_DISTRO}/setup.bash

export DIST="${ROS_DISTRO}"
export RELEASE_TAG="qualifiers_round_release"

# #############################################################################
#
# Installation of the ROS system (based on the docker image)
#

# Base system tools
sudo apt update &&
   sudo apt install -y \
      build-essential \
      ccache \
      clang-format \
      cmake \
      cppcheck \
      curl \
      git \
      locate \
      lsb-release \
      mc \
      pkg-config \
      python3-pip \
      software-properties-common \
      sudo \
      tmux \
      wget &&
   sudo apt clean

export DEBIAN_FRONTEND=noninteractive &&
   sudo apt update &&
   sudo apt install -y \
      tzdata &&
   ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime &&
   dpkg-reconfigure --frontend noninteractive tzdata &&
   sudo apt clean

# Base ROS melodic system
sudo apt update &&
   sudo apt install -y \
      python-rosdep \
      python-catkin-lint \
      python-catkin-tools \
      ros-${DIST}-desktop-full &&
   rosdep init &&
   sudo apt clean

#
# Additional GEAR dependencies obtained running
# rosdep install -si --reinstall --from-path src/ | cut -f7- -d" " | sort

sudo apt update &&
   sudo apt install -y \
      libboost-all-dev \
      libeigen3-dev \
      libncurses5-dev \
      libzmq3-dev \
      python-catkin-lint \
      python-rospkg \
      ros-melodic-effort-controllers \
      ros-melodic-joint-state-publisher-gui \
      ros-melodic-moveit-fake-controller-manager \
      ros-melodic-moveit-kinematics \
      ros-melodic-moveit-planners-ompl \
      ros-melodic-moveit-resources-panda-moveit-config \
      ros-melodic-moveit-ros-move-group \
      ros-melodic-moveit-ros-visualization \
      ros-melodic-moveit-setup-assistant \
      ros-melodic-moveit-visual-tools &&
   sudo apt clean

# Additional ros-control dependencies not installed by the previous stages
sudo apt update &&
   sudo apt install -y \
      "ros-melodic-ros-control*" \
      "ros-melodic-control*" \
      "ros-melodic-gazebo-ros-control*" \
      ros-melodic-moveit-simple-controller-manager \
      ros-melodic-moveit-commander &&
   sudo apt clean

RUN rosdep update

# #############################################################################
#
# Installation of the ROS repository
#

# Create a catkin workspace
mkdir -p ~/tij_team_ws/src/

# Fetch the competition code
mkdir -p ~/tij_team_ws/src
cd ~/tij_team_ws/src
git clone https://github.com/glpuga/ariac2021ws.git
cd ariac2021ws
git checkout $RELEASE_TAG

# Download the extra code (using submodules for this is a pain)
mkdir -p ~/tij_team_ws/src/ariac2021ws/external
cd ~/tij_team_ws/src/ariac2021ws/external
git clone https://github.com/glpuga/ARIAC.git
cd ARIAC
# Checkout the correct branch
git checkout $RELEASE_TAG

# Build the competition code
cd ~/tij_team_ws
catkin_make
