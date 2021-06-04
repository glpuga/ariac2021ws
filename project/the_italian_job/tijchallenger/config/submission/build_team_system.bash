#!/usr/bin/env bash

#
# Installation of the ROS system
#

source /opt/ros/${ROS_DISTRO}/setup.bash

export DIST="${ROS_DISTRO}"

# Base system tools
apt update \
 && apt install -y \
    build-essential \
    cppcheck \
    curl \
    cmake \
    lsb-release \
    gdb \
    git \
    mercurial \
    python3-dbg \
    python3-pip \
    python3-venv \
    ruby \
    software-properties-common \
    sudo \
    vim \
    wget \
    libeigen3-dev \
    pkg-config \
    protobuf-compiler \
    mc \
    locate \
    xterm \
    clang-format \
 && apt clean

export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# install the updated ROS 1 key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# install the updated ROS 2 key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

apt update \
 && apt install -y \
    python-rosdep \
    ros-${DIST}-controller-manager \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-industrial-core \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joint-state-publisher-gui \
    ros-${DIST}-joint-state-controller \
    ros-${DIST}-joint-trajectory-action \
    ros-${DIST}-joint-trajectory-action-tools \
    ros-${DIST}-joint-trajectory-controller \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-moveit-fake-controller-manager \
    ros-${DIST}-moveit-kinematics \
    ros-${DIST}-moveit-opw-kinematics-plugin \
    ros-${DIST}-moveit-planners-ompl \
    ros-${DIST}-moveit-ros-move-group \
    ros-${DIST}-moveit-ros-visualization \
    ros-${DIST}-moveit-setup-assistant \
    ros-${DIST}-moveit-simple-controller-manager \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-ros-base \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-rviz \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-topic-tools \
    ros-${DIST}-trac-ik \
    ros-${DIST}-industrial-core \
    ros-${DIST}-trac-ik-kinematics-plugin \
    ros-${DIST}-velodyne-simulator \
    ros-${DIST}-xacro \
 && rosdep init \
 && apt clean

# Add extra ROS packages here
apt update \
 && apt install -y \
    ros-${DIST}-image-proc \
    ros-${DIST}-vision-opencv \
    ros-melodic-moveit \
    ros-melodic-moveit-visual-tools \
 && apt clean

# Install extra tools tools
apt update \
 && apt install -y \
    tmux \
    ssh \
 && apt clean

rosdep update

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
git checkout final_branch

# Download the extra code (using submodules for this is a pain)
mkdir -p ~/tij_team_ws/src/ariac2021ws/external
cd ~/tij_team_ws/src/ariac2021ws/external
git clone https://github.com/glpuga/ARIAC.git
cd ARIAC
git checkout final_branch

# Build the competition code
cd ~/tij_team_ws
catkin_make
