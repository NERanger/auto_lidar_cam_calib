cmake_minimum_required(VERSION 3.1)
project(auto_lidar_cam_calib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # using GCC
  message(STATUS "Detect GNU compiler")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # using Visual Studio C++
  message(STATUS "Detect MSVC compiler")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
endif()

# Eigen3
find_package(Eigen3 CONFIG REQUIRED)

# glog
find_package(glog CONFIG REQUIRED)

# PCL
find_package(PCL 1.3 CONFIG REQUIRED COMPONENTS common io filters)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV CONFIG REQUIRED)

add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(samples)