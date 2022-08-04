cmake_minimum_required(VERSION 3.0.2)
project(tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(homography homography.cpp)
target_link_libraries(homography ${OpenCV_LIBS} ${catkin_LIBRARIES})

