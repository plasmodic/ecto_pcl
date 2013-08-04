# create the cells to output messages
include_directories(SYSTEM
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(ecto_pcl_ros INSTALL
           module.cpp
           pcl_bridge.cpp
)

link_ecto(ecto_pcl_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
