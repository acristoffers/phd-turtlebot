#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get upgrade -y
apt-get dist-upgrade -y

apt-get install -yq \
    "ros-noetic-turtlebot3*" \
    bash-completion \
    black \
    curl \
    git \
    ipython3 \
    locales \
    neovim \
    python3-{pip,numpy,scipy} \
    ros-noetic-desktop-full \
    ros-noetic-{dynamixel-sdk,slam-gmapping,dwa-local-planner} \
    tmux

pip3 install setuptools==58.2.0
{
    echo "source /usr/share/bash-completion/bash_completion"
    echo "source /opt/ros/noetic/setup.bash"
    echo "source /workspace/devel/setup.bash"
    echo "export USER=\$(whoami)"
    echo "cd /workspace"
} >>/etc/bash.bashrc

locale-gen en_US.UTF-8
localedef -f UTF-8 -i en_US -A /usr/share/locale/locale.alias -c en_US.UTF-8
