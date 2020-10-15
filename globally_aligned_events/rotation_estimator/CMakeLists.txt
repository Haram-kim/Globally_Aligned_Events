cmake_minimum_required(VERSION 2.8.3)
project(rotation_estimator)

set( CMAKE_BUILD_TYPE Release )
# set( CMAKE_BUILD_TYPE Debug )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -m64 -flto -ffast-math -msse3 -march=native -mfpmath=sse")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  dvs_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/utils
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/utils/System.cpp
  src/utils/Database.cpp
  src/utils/Undistortion.cpp
  src/utils/Estimator.cpp
  src/utils/numerics.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

add_executable(rotation_estimation src/main.cpp)
target_link_libraries(rotation_estimation
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
)