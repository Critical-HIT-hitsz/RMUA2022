cmake_minimum_required(VERSION 3.0.2)
project(sentry_beta)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CUDA_MIN_VERSION "7.0")
find_package(CUDA ${CUDA_MIN_VERSION} REQUIRED)
add_definitions("-DCUDA_ENABLED")
CUDA_SELECT_NVCC_ARCH_FLAGS(CUDA_ARCH_FLAGS ${CUDA_ARCHS})

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH_FLAGS}")

# Fix for some combinations of CUDA and GCC (e.g. under Ubuntu 16.04).
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -D_FORCE_INLINES")
# Do not show warnings if the architectures are deprecated.
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Wno-deprecated-gpu-targets")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  robot_msgs
  image_transport
)

set(CUDA_TOOLKIT_ROOT_DIR     "/usr/local/cuda")
set(CUDNN_DIR    "/usr/local/cuda/lib64")
set(TENSORRT_DIR "/home/rmua1/TensorRT-8.2.4.2")
include_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${TENSORRT_DIR}/include
    ${CUDNN_DIR}/include
    sort
    tensorrt
    visualize
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
    ${TENSORRT_DIR}/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDNN_DIR}/lib
)

find_package(OpenCV REQUIRED)

catkin_package(

)

add_library(sort
      sort/rsort.cpp 
      # sort/tracker.cpp
      sort/Hungarian.cpp
      sort/pv_kalman_filter.cpp
)

cuda_add_executable(sentry_beta_node sentry_node.cpp sort/trrt_detector.cpp )
add_dependencies(sentry_beta_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sentry_beta_node 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
  sort
  ${CMAKE_CURRENT_SOURCE_DIR}/libtensorrt_pro.so
  ${EIGEN3_LIBRARIES}
)

