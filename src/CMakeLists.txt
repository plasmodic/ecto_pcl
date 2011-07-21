include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(ecto_pcl 
  ecto_pcl.cpp
  features/NormalEstimation.cpp
  filters/ExtractIndices.cpp
  filters/PassThrough.cpp
  filters/ProjectInliers.cpp
  filters/VoxelGrid.cpp
  segmentation/ExtractPolygonalPrismData.cpp
  segmentation/EuclideanClusterExtraction.cpp
  segmentation/SACSegmentation.cpp
  segmentation/SACSegmentationFromNormals.cpp
  surface/ConvexHull.cpp
  CloudViewer.cpp
  KinectGrabber.cpp
  )

link_ecto(ecto_pcl ${PCL_LIBRARIES})
install_ecto_module(ecto_pcl)
