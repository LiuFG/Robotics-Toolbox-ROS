cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_show)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visualization_simple depth2cloud.cpp)
target_link_libraries (visualization_simple ${PCL_LIBRARIES} ${Opencv_LIBS})