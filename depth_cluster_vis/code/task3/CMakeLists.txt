cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
#SET(CMAKE_C_COMPILER g++)
#add_compile_options(-std=c++11)
project(pcl_show)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV)
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (visualization_simple multi_visualize.cpp)
target_link_libraries (visualization_simple ${PCL_LIBRARIES} ${Opencv_LIBS})
