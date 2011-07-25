cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

message(STATUS "--- ${PROJECT_NAME} ---")

include(rosbuild_lite.cmake)
find_ros_package(pcl)
find_ros_package(sensor_msgs)

find_package(ecto REQUIRED)

if(pcl_NOTFOUND)
  message("**\n** Disabling build of ${PROJECT_NAME} due to missing dependency PCL\n**")
  return()
endif()

if (NOT ecto_kitchen_SOURCE_DIR)
  set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
  ecto_python_env_gen(${CMAKE_BINARY_DIR})
endif()

find_package(Boost COMPONENTS signals thread)

message(STATUS "pcl includes :\n    ${pcl_INCLUDES}")
message(STATUS "pcl library  :\n    ${pcl_LIBRARIES}")

include_directories(${pcl_INCLUDES})
include_directories(${sensor_msgs_INCLUDES})

include_directories(include)
add_subdirectory(src)
