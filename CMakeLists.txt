cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

include(rosbuild_lite.cmake)

find_ros_package(pcl)
find_ros_package(sensor_msgs)

find_package(ecto REQUIRED)
#find_package(PCL REQUIRED )
find_package(OpenCV 2.3.1 EXACT REQUIRED)

#if(NOT PCL_FOUND)
#return()
#endif()

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
ecto_python_env_gen(${CMAKE_BINARY_DIR})
find_package(Boost COMPONENTS signals thread)

message(STATUS "pcl includes :\n    ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl library  :\n    ${PCL_LIBRARIES}")

include_directories(${pcl_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})

include_directories(include)

add_subdirectory(src)
