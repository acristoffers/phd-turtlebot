#!/usr/bin/env bash

apt-get autoremove --purge -y
rm -rf /tmp/*
rm -rf /root/init*

{
    echo ""
    echo "export TURTLEBOT3_MODEL=waffle_pi"
    echo "export ROS_MASTER_URI='http://TurtleBot-Robot1:11311'"
    echo "export ROS_HOSTNAME=Alan-NixOS-SteamDeck"
} >> ~/.bashrc

printf "10.10.0.11\t\tTurtleBot-Robot1" >> /etc/hosts
