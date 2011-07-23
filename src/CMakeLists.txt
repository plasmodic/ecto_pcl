include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(ecto_pcl 
  ecto_pcl.cpp
  features/NormalEstimation.cpp
  filters/ExtractIndices.cpp
  filters/PassThrough.cpp
  filters/ProjectInliers.cpp
  filters/VoxelGrid.cpp
  #io/CloudViewer.cpp
  #io/KinectGrabber.cpp
  #io/PCDReader.cpp
  segmentation/ExtractPolygonalPrismData.cpp
  segmentation/EuclideanClusterExtraction.cpp
  segmentation/SACSegmentation.cpp
  segmentation/SACSegmentationFromNormals.cpp
  #segmentation/SegmentDifferences.cpp
  surface/ConvexHull.cpp
  )

link_ecto(ecto_pcl
    ${sensor_msgs_LIBRARIES}
    ${pcl_LIBRARIES}
    )
install_ecto_module(ecto_pcl)
add_subdirectory(ros)