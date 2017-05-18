cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(txt_to_pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (txt_to_pcd main.cpp)
target_link_libraries (txt_to_pcd ${PCL_LIBRARIES})
