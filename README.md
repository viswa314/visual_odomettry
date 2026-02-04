![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-1.5.0-blue.svg)
![Last Updated](https://img.shields.io/badge/Last_Updated-31.01.2026-brightgreen.svg)

# ROS2 ORB SLAM3 V1.0 package 

A ROS2 package for ORB SLAM3 V1.0, maintained by **viswa**. Focus is on native integration with ROS2 ecosystem. This is the `main/humble` branch which only supports ROS 2 Humble. Switch over to `jazzy` branch to use with ROS 2 Jazzy.

My goal is to provide a robust and easy-to-use starting point for developers in using ORB SLAM3 framework in their ROS 2 projects. This package simplifies the integration of ORB SLAM3 with standard ROS 2 messaging and infrastructure.

## 📺 Demonstration
## 📺 Demonstration
<video src="media/Task%205%20final.webm" controls="controls" style="max-width: 100%;">
</video>
*ORB-SLAM3 running in Monocular mode on EuRoC MAV dataset.*

If you find this work useful, please consider citing the original ORB-SLAM3 paper:

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 }
```

## 0. Preamble

* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library.
* Optimized for ROS 2 Humble and Jazzy.
* Includes support for Monocular, Stereo, and RGB-D modes (see roadmap).
* Comes with a small test image sequence from EuRoC MAV dataset (MH05) for quick verification.

## Testing platforms

1. Ubuntu 22.04 LTS (Jammy Jellyfish) and ROS 2 Humble Hawksbill (LTS)
2. Ubuntu 24.04 LTS (Noble Numbat) and ROS 2 Jazzy Jalisco (LTS)

## 1. Prerequisites

Start by installing the following prerequisites:

### Eigen3

```bash
sudo apt install libeigen3-dev
```

### Pangolin and configuring dynamic library path

Install Pangolin system-wide:

#### Install Pangolin

```bash
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j4
sudo cmake --install build
```

#### Configure dynamic library

Ensure `/usr/local/lib` is in your `LD_LIBRARY_PATH`:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
sudo ldconfig
```

## 2. Installation

Follow the steps below to create a workspace and build the package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/viswa/ros2_orb_slam3.git
cd ..
rosdep install -r --from-paths src --ignore-src -y
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 4. Launching with ROS 2 Launch (Recommended)

You can now start both the SLAM node and the image driver with a single command:

```bash
source install/setup.bash
ros2 launch ros2_orb_slam3 mono_launch.py settings_name:=EuRoC image_seq:=sample_euroc_MH05
```

## 5. Visualization and Topics

The SLAM node now publishes standard ROS 2 topics and TF transforms:

### Topics
| Topic | Data Type | Description |
| --- | --- | --- |
| `/mono_slam_cpp/pose` | `geometry_msgs/PoseStamped` | Current camera pose in `map` frame |
| `/mono_slam_cpp/odom` | `nav_msgs/Odometry` | Current camera pose / odometry |
| `/tf` | `tf2_msgs/TFMessage` | Transform from `map` to `camera_link` |

### Using RViz2
Open RViz2:
```bash
ros2 run rviz2 rviz2
```
1. Set Fixed Frame to `map`.
2. Add a **Pose** display subscribing to `/mono_slam_cpp/pose`.
3. Add a **TF** display to see the `camera_link` frame moving.

## roadmap

- [x] Monocular mode example
- [ ] Stereo mode example
- [ ] RGBD mode example
- [ ] IMU integration improvements

---
Developed and maintained by **viswa**
Contact: [viswagandamalla@gmail.com](mailto:viswagandamalla@gmail.com)
Last Updated: 31.01.2026
