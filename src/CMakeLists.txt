include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(ecto_pcl
  ecto_pcl.cpp
  features/NormalEstimation.cpp
  features/FPFHEstimation.cpp
  features/PFHEstimation.cpp
  features/VFHEstimation.cpp
  filters/ExtractIndices.cpp
  filters/PassThrough.cpp
  filters/ProjectInliers.cpp
  filters/StatisticalOutlier.cpp
  filters/RadiusOutlier.cpp
  filters/VoxelGrid.cpp
  io/CloudViewer.cpp
  # io/KinectGrabber.cpp
  io/MergeClouds.cpp
  io/NiConverters.cpp
  io/PCDReader.cpp
  io/PCDWriter.cpp
  io/PLYWriter.cpp
  segmentation/ColorizeClusters.cpp
  segmentation/ExtractPolygonalPrismData.cpp
  segmentation/ExtractLargestCluster.cpp
  segmentation/EuclideanClusterExtraction.cpp
  segmentation/SACSegmentation.cpp
  segmentation/SACSegmentationFromNormals.cpp
  surface/ConvexHull.cpp
  )

set_source_files_properties(io/CloudViewer.cpp
  PROPERTIES
  COMPILE_FLAGS -Wno-deprecated
  )

link_ecto(ecto_pcl
    ${pcl_LIBRARIES}
    )

install_ecto_module(ecto_pcl)
add_subdirectory(ros)
