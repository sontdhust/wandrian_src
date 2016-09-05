#!/usr/bin/env bash

# Check args
if [ "$#" -ne 1 ]; then
  echo "usage: ./run.sh IMAGE_NAME"
  return 1
fi

# Get this script's path
pushd `dirname $0` > /dev/null
WORKSPACE_PATH="$(dirname "$(dirname "$(pwd)")")"
popd > /dev/null

set -e

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Run the container with shared X11
docker run \
  --net=host \
  -e SHELL \
  -e DISPLAY \
  -e DOCKER=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$WORKSPACE_PATH:$WORKSPACE_PATH:rw" \
  -v "$(pwd)/.ros:$HOME/.ros:rw" \
  -v "$(pwd)/.gazebo:$HOME/.gazebo:rw" \
  -v "${XSOCK}:${XSOCK}:rw" \
  -v "${XAUTH}:${XAUTH}:rw" \
  -it $1 $SHELL
