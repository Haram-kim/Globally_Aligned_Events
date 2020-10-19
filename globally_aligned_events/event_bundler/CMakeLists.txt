cmake_minimum_required(VERSION 2.8.3)
project(event_bundler)

set( CMAKE_BUILD_TYPE Release )
# set( CMAKE_BUILD_TYPE Debug )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -m64 -flto -ffast-math -msse3 -march=native -mfpmath=sse")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dvs_msgs
)

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
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/EventBundler.cpp
)

add_executable(bundler src/main.cpp)
target_link_libraries(bundler
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
