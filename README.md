# Curb2Door

This repository contains code for the NYU Tandon Undergraduate Summer Research Program (UGSRP) Curb2Door Project. The project entails the collection and processing of 360 images and LiDAR pointcloud data in order to create a simulated environment viable for training autonomous navigation systems. 

---
## Table of Contents
1. [Installation](#Installation)
2. [Recording Data](#Recording-Data)
3. [Processing Data](#Processing-Data)
4. [Exporting Data](#Exporting-Data)

---
## Installation
### Dependencies
The package files, along with the necessary submodules, can be installed with the following command. Make sure to run it in the ```catkin_ws/src``` folder. 
```
git clone --recurse-submodules https://github.com/Abanesjo/Curb2Door
```
Note that each of the submodules still requires their individual dependencies as listed as follows:
- [insta360_ros_driver](https://github.com/ai4ce/insta360_ros_driver)
- [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2.git)
- [r3live](https://github.com/hku-mars/r3live)

## Recording Data
To begin recording data, the launch file can be used.
```
roslaunch curb2door record.launch
```
| Argument | Values (Default) | Description |
| ----------- | ----------- | -----------  | 
| bag_path | <user_defined> ($(curb2door)/bag_raw) | Directory to save raw recorded bag files |
| record | True/False (True) | Record data to bag file|
| rviz | True/False (False) | Show live data using RViz |

For example:
```
roslaunch curb2door record.launch bag_path:=/home/bag_raw/ record:=True rviz:=False
```
## Processing Data 
```
```
## Exporting Data
```
```