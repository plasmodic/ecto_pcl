# create the cells to output messages
rosbuild_lite_init()

find_package(ROS QUIET)
if (NOT ROS_FOUND)
  return()
endif()

if (ROS_ELECTRIC_FOUND)
  find_ros_package(sensor_msgs)
else()
  find_package(ROS REQUIRED sensor_msgs)
endif()

include_directories(SYSTEM
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(ecto_pcl_ros INSTALL
           module.cpp
           pcl_bridge.cpp
)

link_ecto(ecto_pcl_ros ${PCL_LIBRARIES})
