#!/usr/bin/env bash

#
# Main runner only meant to be be used on for running build and test in CI
# For regular use of the docker container, use ./run.sh
#

IMAGE_NAME="ariac2021_devel_env"

RUNTIME="runc"

LOCAL_REPO_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../" >/dev/null 2>&1 && pwd )"

USERID=$(id -u)
GROUPID=$(id -g)

mkdir -p $HOME/.ci_docker_ccache

docker run \
  --mount type=bind,source=$HOME/.ci_docker_ccache,target=/home/developer/.ccache \
  --mount type=bind,source=${LOCAL_REPO_PATH},target=/home/developer/ws/src/workspace \
  --runtime=$RUNTIME \
  --rm \
  -u $USERID:$GROUPID \
  $IMAGE_NAME \
  /bin/bash -c " \
    source /opt/ros/melodic/setup.bash && \
    export PATH=/usr/lib/ccache:\$PATH && \
    catkin_make && \
    catkin_make run_tests && \
    catkin_test_results
  "
