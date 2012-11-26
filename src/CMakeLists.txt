include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (ROS_GROOVY_FOUND)
find_package(catkin REQUIRED roscpp sensor_msgs)

include_directories(SYSTEM ${sensor_msgs_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
endif()

set(ECTO_PCL_FILES
  ecto_pcl.cpp
  features/NormalEstimation.cpp
  features/FPFHEstimation.cpp
  features/PFHEstimation.cpp
  features/VFHEstimation.cpp
  filters/Cropper.cpp
  filters/ExtractIndices.cpp
  filters/PassThrough.cpp
  filters/ProjectInliers.cpp
  filters/StatisticalOutlier.cpp
  filters/RadiusOutlier.cpp
  filters/VoxelGrid.cpp
  io/CloudViewer.cpp
#  io/KinectGrabber.cpp
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
  surface/MovingLeastSquares.cpp
)

if (NOT ROS_GROOVY_FOUND)
  set(ECTO_PCL_FILES ${ECTO_PCL_FILES} io/PCDWriter.cpp)
endif()

ectomodule(ecto_pcl INSTALL ${ECTO_PCL_FILES})

set_source_files_properties(io/CloudViewer.cpp
  PROPERTIES
  COMPILE_FLAGS -Wno-deprecated
  )

if (ROS_GROOVY_FOUND)
link_ecto(ecto_pcl ${PCL_LIBRARIES}
                   ${roscpp_LIBRARIES}
)
else()
link_ecto(ecto_pcl
    ${PCL_LIBRARIES}
)
endif()

add_subdirectory(ros)
