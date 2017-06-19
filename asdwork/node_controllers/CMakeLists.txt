cmake_minimum_required(VERSION 2.8.3)
project(node_controllers)

find_package(catkin REQUIRED COMPONENTS
  nao_interaction_msgs
  nao_msgs
  naoqi_bridge_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES node_controllers
#  CATKIN_DEPENDS nao_interaction_msgs nao_msgs naoqi_bridge_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wave_nao_work src/wave_working.cpp)
add_dependencies(wave_nao_work ${catkin_EXPORTED_TARGETS})
target_link_libraries(wave_nao_work
  ${catkin_LIBRARIES}
)