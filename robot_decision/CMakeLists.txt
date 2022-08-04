cmake_minimum_required(VERSION 2.8.3)
project(robot_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")
#add_subdirectory(./backward-cpp)
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  robot_msgs
  tf
  roslib
  # robot_localization
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

add_executable(robot_decision
  decision_node.cpp
    #${BACKWARD_ENABLE}
  )

add_dependencies(robot_decision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_decision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #dw
)
#add_backward(robot_decision)
