cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_ground_filtering)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (project_ground_filtering myGroundFiltering.cpp)
target_link_libraries (project_ground_filtering ${PCL_LIBRARIES})
