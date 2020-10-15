cmake_minimum_required(VERSION 2.8.3)
project(event_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
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
)

add_library(event SHARED src/Event.cpp)
target_link_libraries(event
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(image SHARED src/Image.cpp)
target_link_libraries(image
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(imu SHARED src/IMU.cpp)
target_link_libraries(imu
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(event_publisher src/main.cpp)
target_link_libraries(event_publisher
    ${OpenCV_LIBS}
    event
    image
    imu
)