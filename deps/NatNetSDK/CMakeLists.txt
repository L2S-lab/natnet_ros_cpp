cmake_minimum_required(VERSION 3.5)
project(NatNetSDK)

find_package(Boost 1.65 REQUIRED COMPONENTS system thread)
# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Executables

## PacketClient
add_executable(packetClient
  src/main.cpp
 samples/PacketClient/PacketClient.cpp
)
target_link_libraries(packetClient
  ${Boost_LIBRARIES}
)

## SampleClient
include_directories(include)
link_directories(lib)
add_executable(sampleClient
  samples/SampleClient/SampleClient.cpp
)
target_link_libraries(sampleClient
  NatNet
)
