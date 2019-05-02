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
5) Remove the overlaying layers of the box and plane point cloud. 
6) Save the Point cloud using a service

## Output

The output from the first node of synthesizing data is shown on the left and the segmented cube is shown on the right. The terminal shows the random angle given on the left and the orientation and the center of the cube predicted on the right:

<p align="center">
<img src="https://github.com/anirudhtopiwala/Box_Point_Ploud_Creation_and_Detection/blob/master/output/box_detector.gif">
</p>

The video [here](https://youtu.be/5SLp6oLASQ4) is much clearer.

## NEW IMPROVEMENTS (All Tasks Completed !!!)
**1)The Cube is now Hollow**
As I realized later, any camera would not able to give points inside the cube when reading point cloud data in realistic situations and therefore, code is fixed to generate a hollow cube.  

**2)Get Orientation of Cube is Fixed**
The part of getting the orientation of cube implemented in the second node is now fixed. We can see the prediction is with a +- 2 degrees accuracy. This can be improved by further removing the noise which was added earlier. 

**3)Remove points from bottom of the cube and the overlaying points of the plane** 
Removed the bottom layer of the cube using filter pass in z direction. Used **kd-trees** to remove the corresponding points from the plane point cloud. The updated simulation can be seen below:
<p align="center">
<img src="https://github.com/anirudhtopiwala/Box_Point_Ploud_Creation_and_Detection/blob/master/output/box_detector_fixed.gif">
</p>

**This marks the end of all tasks in this repository.**

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
roscore
```
In a different terminal run
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

