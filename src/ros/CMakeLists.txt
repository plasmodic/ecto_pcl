# create the cells to output messages
if (NOT ROS_FUERTE_FOUND AND NOT ROS_GROOVY_FOUND)
  return()
endif()

  if (ROS_FUERTE_FOUND)
  find_package(ROS REQUIRED sensor_msgs)
  else()
find_package(catkin REQUIRED sensor_msgs)
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
