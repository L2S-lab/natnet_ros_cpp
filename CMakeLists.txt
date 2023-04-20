cmake_minimum_required(VERSION 3.0.2)
project(natnet_ros_cpp)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2 
  tf2_ros
)
find_package(Boost 1.65 REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

#download Natnet SDK
execute_process(
	COMMAND chmod "+x" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
execute_process(
	COMMAND bash "-c" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Enable C++14 and warnings
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/) 
set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/lib/libNatNet.so)
set(my_include_directories
  include/natnet_ros_cpp
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NATNET_SDK_DIR}/include
)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES natnet_ros_cpp
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${my_include_directories}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/natnet_ros_cpp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Use following flag to build in debug mode : catkin_make -DCMAKE_BUILD_TYPE=Debug

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/natnet_ros.cpp src/internal.cpp src/nn_filter.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${NATNET_LIBRARY_DIR}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_natnet_ros_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
