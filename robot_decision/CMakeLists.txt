cmake_minimum_required(VERSION 2.8.3)
project(robot_decision)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  robot_msgs
  tf
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_base
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include 
  behaviors 
  executor 
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)

add_executable(${PROJECT_NAME}
  decision_node.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
