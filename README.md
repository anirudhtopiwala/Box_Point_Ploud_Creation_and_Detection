# Box_Point_Cloud_Creation_and_Detection
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview
Synthesizing a box point cloud and then detecting it and localizing it. (ROS and PCL)
The Details of the Problem are as follows:
1) Synthesize a cloud of noisy points of a plane (x,y in [-5, 5], z = 0), resolution 0.1m.
2) Synthesize a surface point cloud of a unit box (width, length, and height = 1). Since the box is on the plane, the height of box center z = 0.5m, roll and pitch angles are zero. The box is movable on the plane, so x and y are in [-2, 2], and yaw is [0, pi/2). Again, the resolution is 0.1m.
3) add random noise within +/- 0.02cm on all points. The noise is refreshed at 5Hz.
4) estimated the position and orientation of the unit box
5) Save the Point cloud using a service

## Output

The output from the first node of synthesizing data is as follows:

<p align="center">
<img src="https://github.com/anirudhtopiwala/Box_Point_Ploud_Creation_and_Detection/blob/master/output/pclcube.gif">
</p>

## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/anirudhtopiwala/Box_Point_Ploud_Creation_and_Detection.git
cd ..
catkin_make
```
If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/anirudhtopiwala/Box_Point_Ploud_Creation_and_Detection.git
cd ..
catkin_make
```

## Run Instructions

After following the build instructions:
Go to your workspace in terminal
```
source devel/setup.bash
rosrun box_detector plt 
```
In a different terminal run
```
source devel/setup.bash
rosrun box_detector seg
```
## Calling the Service
**Note this will affect your local storage**
The service to save the detected box point cloud can be run as follows:
```
rosservice call /write_to_file 
```
The file will be outside the src folder in the base directory.

## General Notes and Things_TO-DO

1) **Remove points from bottom of the cube and the overlaying points of the plane** .
Things tried: tried to remove all the points of cube when z=0, by defining them as NaN, which worked in rviz visualization but when tried to get centroid, the nan values created a problem. Therefore not implemented for now. 

2) **Get Orientation of Cube**
In the current implementation to get orientation of cube, it works when cube angle rotation is less than 45 degrees. This is because, the current approach always tries to get the top right corner point of a layer of cube (square), and when the rotation is more than 45, the algorithm will mistakenly identify the right bottom point and therefore giving inaccurate values.   
A more efficient approach can be taking an image of cube from above and getting the angle using image processing. This is not tried yet.

