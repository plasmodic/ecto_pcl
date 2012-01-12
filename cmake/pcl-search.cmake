
option(ECTO_PCL_STANDALONE "Use standalone PCL" ON)

#
# transitional:  first try to find 1.4.0 or greater, and if so set the option OFF.
#
if (ECTO_PCL_STANDALONE)

  message(STATUS "+ Looking for PCL 1.4.0...")
  find_package(PCL 1.4.0 EXACT QUIET)
  if (PCL_FOUND)
    message(STATUS "+ Found PCL Version ${PCL_VERSION} with config from ${PCL_CONFIG}. ")
    message(STATUS "+ Setting ECTO_PCL_STANDALONE to ON.")
    message(STATUS "+ *********************************************************************")
    message(STATUS "+ This might mean your ecto_ros and ecto_pcl will fight over the definition")
    message(STATUS "+ of sensor_msgs::PointCloud2 and cause demons to come out of each others")
    message(STATUS "+ noses.")
    message(STATUS "+ *********************************************************************")
    #
    #  This is nasty, but they don't have usable macros to compare versions until
    #  some release > 1.4.0
    #
    add_definitions(-DPCL_VERSION_GE_140=1)
    return()
  else()
    message(STATUS "Didn't find 1.4.0.  Trying via ROS.")
    set(ECTO_PCL_STANDALONE OFF)
  endif()

  rosbuild_lite_init() # found in rosbuild_lite
  if (NOT ROS_FOUND)#rosbuild_lite_init sets this
    message(STATUS "+ Didn't find ROS via the deprecated rosbuild_lite... Will try older versions.")
    message(STATUS "+ If you wanted this to work, is ROS_ROOT set in your environment?")
    set(ECTO_PCL_STANDALONE ON)
  else()
    message(STATUS "+ Found ROS.")
    set(ECTO_PCL_STANDALONE OFF)
  endif()
endif()

set(CMAKE_MODULE_PATH ${ecto_pcl_SOURCE_DIR}/cmake)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

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
  set(PCL_INCLUDE_DIRS ${pcl_INCLUDE_DIRS})
  set(PCL_LIBRARIES ${pcl_LIBRARIES})

  find_ros_package(sensor_msgs)
  include_directories(${sensor_msgs_INCLUDE_DIRS})

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

endif()

