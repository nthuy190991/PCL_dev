cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_to_ply)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_to_ply main.cpp)
target_link_libraries (pcd_to_ply ${PCL_LIBRARIES})
