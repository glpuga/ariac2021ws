# Ubuntu 18.04 with nvidia-docker2 beta opengl support.
ARG BASEIMG=ubuntu:bionic
FROM $BASEIMG

# Set ROS distribution
ARG DIST=melodic

# Set Gazebo verison
ARG GAZ=gazebo9

# Base system tools
RUN apt update \
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
 && apt clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Missing nvidia libraries
RUN sudo apt update \
 && sudo apt install -y \
    libnvidia-gl-440 \
 && apt clean

# Base ROS melodic system
COPY keys/ros.key /tmp/ros.key
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key add /tmp/ros.key \
 && apt update \
 && apt install -y \
    python-rosdep \
    ros-${DIST}-controller-manager \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joint-state-publisher-gui \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-moveit-fake-controller-manager \
    ros-${DIST}-moveit-kinematics \ 
    ros-${DIST}-moveit-planners-ompl \
    ros-${DIST}-moveit-ros-move-group \
    ros-${DIST}-moveit-ros-visualization \
    ros-${DIST}-moveit-setup-assistant \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-ros-base \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-rviz \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-velodyne-simulator \
 && rosdep init \
 && apt clean

RUN rosdep update

# Get Gazebo 9.
COPY keys/gazebo.key /tmp/gazebo.key
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && apt-key add /tmp/gazebo.key \
 && apt update \
 && apt install -y \
    qtbase5-dev \
    libgles2-mesa-dev \
    ${GAZ} \
    lib${GAZ}-dev \
    ros-${DIST}-gazebo-plugins \
    ros-${DIST}-gazebo-ros \
    ros-${DIST}-xacro \
 && apt clean
# This seems to be needed to launch Gazebo on this setup
# or it fails to launch with a symbol error on gzserver
RUN apt upgrade -y libignition-math2

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

RUN mkdir -p /home/$USER/ws/src \
 && chown -R $USER: home/$USER/ws

# Add extra ROS packages here
RUN apt update \
 && apt install -y \
    python-rosdep \
    ros-${DIST}-image-proc \
    ros-${DIST}-vision-opencv \
    ros-${DIST}-joint-state-controller \
 && apt clean

# Install extra tools tools
RUN apt update \
 && apt install -y \
    mc \
    locate \
    xterm \
    tmux \
 && apt clean 

# Commands below run as the developer user.
USER $USER:$GROUP

RUN rosdep update

# When running a container start in the developer's home folder.
WORKDIR /home/$USER

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'

WORKDIR /home/$USER/ws

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
