#!/usr/bin/env bash

xhost +SI:localuser:$(id -un)
podman run -ti --rm \
    -e DISPLAY \
    -v $XAUTHORITY \
    -v "$PWD/workspace:/workspace:rw" \
    -v /dev/dri \
    --device /dev/dri/card0 \
    --device /dev/dri/renderD128 \
    --group-add=keep-groups \
    --net=host \
    ros-noetic /bin/bash
