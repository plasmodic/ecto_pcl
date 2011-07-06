cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

find_package(ecto REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 2.3.1 EXACT REQUIRED)

if(NOT PCL_FOUND)
return()
endif()

find_package(Boost COMPONENTS signals thread)

message(STATUS "pcl includes :\n    ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl library  :\n    ${PCL_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS})

add_subdirectory(src)
