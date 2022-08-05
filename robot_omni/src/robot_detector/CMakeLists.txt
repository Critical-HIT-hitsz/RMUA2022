
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O0 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")
file(GLOB_RECURSE cuda_srcs ${PROJECT_SOURCE_DIR}/include/robot_detector/*.cu)

cuda_add_library(cucodes SHARED ${cuda_srcs})
target_link_libraries(cucodes nvinfer nvonnxparser)
target_link_libraries(cucodes cuda cublas cudart cudnn)

add_library (robot_detector_lib SHARED RobotDetector.cpp)

target_link_libraries(robot_detector_lib  
                       ${OpenCV_LIBS}
                       cucodes)

