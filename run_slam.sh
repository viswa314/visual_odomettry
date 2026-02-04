#!/bin/bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/viswa/ros2_orb_slam3/Pangolin/build
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting C++ Node..."
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp &
CPP_PID=$!

sleep 5

echo "Starting Python Driver..."
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05 &
PY_PID=$!

sleep 10

echo "Checking status..."
ps -p $CPP_PID
ps -p $PY_PID

kill $CPP_PID $PY_PID
