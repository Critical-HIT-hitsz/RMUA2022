project(kalman_filter)

find_package(Eigen3 REQUIRED)

add_library(kalman_filter
  SHARED
  kalman_filter.cpp
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )
