cmake_minimum_required(VERSION 2.8.3)
project(optitrack_sdk)

set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
  # Options: Debug, Release, MinSizeRel, RelWithDebInfo
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_library(optitrack_sdk SHARED optitrack_client.cpp udp_socket.cpp)
target_include_directories(optitrack_sdk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
