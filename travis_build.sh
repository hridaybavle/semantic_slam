#!/usr/bin/env bash

mkdir -p /root/semantic_slam_build_ws/src
cd /root/semantic_slam_build_ws/src
git clone https://github.com/hridaybavle/semantic_slam.git 
source /opt/ros/kinetic/setup.bash
cd .. && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
