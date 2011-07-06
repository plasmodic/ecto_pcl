include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(ecto_pcl 
  ecto_pcl.cpp
  VoxelGrid.cpp
  CloudViewer.cpp
  KinectGrabber.cpp
  )

link_ecto(ecto_pcl ${PCL_LIBRARIES})
install_ecto_module(ecto_pcl)
