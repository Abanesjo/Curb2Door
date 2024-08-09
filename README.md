# Curb2Door

This repository contains code for the NYU Tandon Undergraduate Summer Research Program (UGSRP) Curb2Door Project. The project entails the collection and processing of 360 images and LiDAR pointcloud data in order to create a simulated environment viable for training autonomous navigation systems. 

---
## Table of Contents
1. [Installation](#Installation)
2. [Recording Data](#Recording-Data)
3. [Processing Data](#Processing-Data)
4. [Exporting Data](#Exporting-Data)
5. [Graphical User Interface](#graphical-user-interface)

---
## Installation
### Dependencies
The package files, along with the necessary submodules, can be installed with the following command. Make sure to run it in the ```catkin_ws/src``` folder. 
```
git clone --recurse-submodules https://github.com/Abanesjo/Curb2Door
```
Note that each of the submodules still requires their individual dependencies as listed as follows:
- [insta360_ros_driver](https://github.com/ai4ce/insta360_ros_driver)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2.git)
- [r3live](https://github.com/hku-mars/r3live)
- [livox_ros_driver_for_R2LIVE](https://github.com/ziv-lin/livox_ros_driver_for_R2LIVE)
- [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration)
- [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git)
- [CameraCalibration](https://github.com/dyfcalid/CameraCalibration)

## Recording Data
For recording data, it is recommended to use the [Graphical User Interface](#graphical-user-interface) to simplify the process. However, it can be done manually as well.

The LiDAR and camera can be activated using the following
```
roslaunch curb2door bringup.launch
```
| Argument | Values (Default) | Description |
| ----------- | ----------- | -----------  | 
| lidar_msg | CustomMsg/PointCloud2 (CustomMsg) | Point Cloud Message Type |
| rviz | True/False (False) | Show live data using RViz |

To begin recording data, the launch file can be used.
```
roslaunch curb2door record.launch
```
| Argument | Values (Default) | Description |
| ----------- | ----------- | -----------  | 
| bag_path | ($(curb2door)/compressed) | Directory to save recorded rosbag files |
| bag_name | (run.bag) | rosbag filename |
## Image Undistortion
Before using R3LIVE, the camera images must first be undistorted.
```
roslaunch curb2door undistort.launch
```
## R3LIVE
R3LIVE can be used to create a 3D map and also estimate odometry.
```
roslaunch curb2oor r3live.launch
```
An example 3D map is shown below.
![r3live_result](docs/r3live_result.png)
## Exporting Data
Using the resultant bag file from R3LIVE, correspondence between camera frame and camera pose can be created. Before running each python file, make sure that the file paths specified within them are correct.
```
cd tools/data_processing
python bag_to_data.py # Converts bag file to odometry and image frames
python data_matching.py #Matches timestamps via indeces
python plot_data.py #Plots odometry data
```
![odom_plot](docs/odom_plot.png)

## Graphical User Interface
A GUI has been developed to expedite data collection and it appears as follows:

<center>Screen for Establishing Remote Connection</center>

![Remote](docs/app_1.png)

<center>Screen for Managing ROS-Related Tasks</center>

![ROS](docs/app_2.png)

<center>Screen for Monitoring Topics</center>

![Analytics](docs/app_3.png)