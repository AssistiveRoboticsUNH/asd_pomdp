cmake_minimum_required(VERSION 2.8.3)
project(asdpomdp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  nao_msgs
  roslib
  rosbag
)

find_package(Boost  REQUIRED COMPONENTS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

catkin_package(

)

#################
## Executables ##
#################

add_executable(asdpomdp src/main.cpp src/pomdp.cpp src/pomdppolicy.cpp src/pomdpexecutor.cpp)
target_link_libraries(asdpomdp ${catkin_LIBRARIES})

add_executable(gaze_observer src/gaze.cpp)
target_link_libraries(gaze_observer ${catkin_LIBRARIES})

add_executable(audio_observer src/audio.cpp)
target_link_libraries(audio_observer ${catkin_LIBRARIES})
