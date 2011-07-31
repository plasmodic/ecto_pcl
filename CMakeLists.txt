cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)
find_package(ecto REQUIRED)
include(rosbuild_lite.cmake)
find_ros_package(pcl)
if(NOT pcl_FOUND)
  message("**\n** Disabling build of ${PROJECT_NAME} due to missing dependency PCL\n**")
  return()
endif()

find_ros_package(sensor_msgs)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
ecto_python_env_gen(${CMAKE_BINARY_DIR}/lib)

find_package(Boost COMPONENTS signals thread)

include_directories(${pcl_INCLUDES})
include_directories(${sensor_msgs_INCLUDES})

include_directories(include)
add_subdirectory(src)
