cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)
find_package(ecto REQUIRED) #brings in rosbuild_lite

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

if (NOT ECTO_PCL_STANDALONE_OPTION) # first time only
  if ("$ENV{ROS_ROOT}" STREQUAL "")
    message(STATUS "+ Setting default value of ECTO_PCL_STANDALONE to ON")
    message(STATUS "+ Because ROS_ROOT is not set in your environment.")
    option(ECTO_PCL_STANDALONE "Use standalone PCL" ON)
  else()
    message(STATUS "+ Setting default value of ECTO_PCL_STANDALONE to OFF")
    message(STATUS "+ Because ROS_ROOT is set to $ENV{ROS_ROOT}")
    option(ECTO_PCL_STANDALONE "Use standalone PCL" OFF)
  endif()
  set(ECTO_PCL_STANDALONE_OPTION YES CACHE INTERNAL "pcl option guard")
endif()

if (NOT ECTO_PCL_STANDALONE)
  if (NOT pcl_FOUND)
    message(STATUS "Attempting to find PCL via ROS environment because ECTO_PCL_STANDALONE is OFF.")
    find_ros_package(pcl)
  endif()

  if(NOT pcl_FOUND)
    message("**\n** Disabling build of ${PROJECT_NAME} due to missing dependency PCL\n**")
    return()
  endif()

  message(STATUS "+ pcl found.")
  include_directories(${pcl_INCLUDES})

  find_ros_package(sensor_msgs)
  include_directories(${sensor_msgs_INCLUDES})

else()

  message(STATUS "+ Searching for standalone PCL because ECTO_PCL_STANDALONE is ON")
  find_package(PCL 1.1)
  if (PCL_FOUND)
    message(STATUS "+ Found PCL version ${PCL_VERSION}")
    message(STATUS "*** You will STILL BUILD AN ECTO_PCL_ROS LIBRARY")
    message(STATUS "*** Which has ecto cell converters from sensor_msgs to point cloud")
    message(STATUS "*** That may be Dangerous, if your ecto_ros compiles against messages")
    message(STATUS "*** with the same name but different size, and both run in the same")
    message(STATUS "*** process simultaneously.")
    message(STATUS "*** See http://en.wikipedia.org/wiki/One_Definition_Rule")
  else()
    message("**\n** Disabling build of ${PROJECT_NAME} due to missing dependency PCL\n**")
    return()
  endif()

  include_directories(${PCL_INCLUDE_DIRS})
  
endif()

find_package(Boost COMPONENTS signals thread)

include_directories(include)
add_subdirectory(src)
