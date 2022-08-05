cmake_minimum_required(VERSION 3.5)
project(omni_detect)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  roslib
  robot_msgs
)
catkin_package()

include_directories(
  "/usr/include/eigen3"
  include/robot_detector
  ${catkin_INCLUDE_DIRS}
)

set(CUDA_GEN_CODE "-gencode=arch=compute_75,code=sm_75")
set(CUDA_TOOLKIT_ROOT_DIR     "/usr/local/cuda-11.3")
set(CUDNN_DIR    "/usr/local/cuda-11.3/lib64")
set(TENSORRT_DIR "/home/rmua3/TensorRT-8.0.1.6") #需要修改为本地TensorRT路径

find_package(CUDA REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${TENSORRT_DIR}/include
    ${CUDNN_DIR}/include
)

link_directories(
    ${TENSORRT_DIR}/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDNN_DIR}/lib
)

add_subdirectory(src)

add_executable(robot_detector_node 
               src/robot_detector_node.cpp  
               )
target_link_libraries(robot_detector_node
                      robot_detector_lib
                      ${catkin_LIBRARIES}
                      )

