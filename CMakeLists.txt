cmake_minimum_required(VERSION 3.0.2)
project(natnet_ros_cpp)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf2 
  tf2_ros
)
#find_package(Boost 1.65 REQUIRED COMPONENTS system thread)
#find_package(Eigen3 REQUIRED)

# Enable C++14 and warnings
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/) 
set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/lib/libNatNet.so)
set(natnet_include_directories
  include/natnet_ros_cpp
  ${NATNET_SDK_DIR}/include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES natnet_ros_cpp
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${natnet_include_directories}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/natnet_ros.cpp src/internal.cpp src/nn_filter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${NATNET_LIBRARY_DIR}
)
