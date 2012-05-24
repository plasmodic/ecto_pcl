# create the cells to output messages
rosbuild_lite_init()

find_package(ecto_ros QUIET)
if (NOT ecto_ros_FOUND)
  return()
endif()
find_package(object_recognition_msgs REQUIRED)

if (ROS_ELECTRIC_FOUND)
  find_ros_package(sensor_msgs)
else()
  find_package(ROS REQUIRED sensor_msgs)
endif()

include_directories(SYSTEM
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(ecto_pcl_ros module.cpp
                        pcl_bridge.cpp
)

link_ecto(ecto_pcl_ros ${PCL_LIBRARIES})
    
install_ecto_module(ecto_pcl_ros)
