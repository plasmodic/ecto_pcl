ectomodule(ecto_pcl_ros 
  module.cpp
  pcl_bridge.cpp
)

link_ecto(ecto_pcl_ros
    ${pcl_LIBRARIES}
    )
    
install_ecto_module(ecto_pcl_ros)
