cmake_minimum_required(VERSION 2.8.3)
project(box_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros  
  PCL REQUIRED
  message_generation
)

add_service_files(
  DIRECTORY
  srv
  FILES
  write_to_file.srv
)


 generate_messages(
   DEPENDENCIES
   std_msgs
   )

catkin_package( INCLUDE_DIRS include
#  LIBRARIES box_detector
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(plt src/main.cpp )
add_executable(seg src/segment.cpp )



## Specify libraries to link a library or executable target against
target_link_libraries(plt
	${catkin_LIBRARIES}
	${PCL_LIBRARIES})

target_link_libraries(seg
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

