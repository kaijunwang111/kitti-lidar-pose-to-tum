cmake_minimum_required(VERSION 2.8)
project(kitti_lidar_pose_to_tum)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

add_executable(camera2lidar camera2lidar.cpp)
