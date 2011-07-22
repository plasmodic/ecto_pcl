#!/usr/bin/env python

"""
This example shows how to extract points corresponding to objects on a table. 

  1) The example downsamples using a VoxelGrid before estimating 
     normals for the downsampled cloud. 
  2) These normals are then used for segmentation using RANSAC. 
  3) Segmentation produces a planar model to which all inliers are 
     projected so that a 2D convex hull can be created. 
  4) We then extract the indices of all points that are above the 
     plane formed by the convex hull. 
  5) Finally, we extract the point cloud corresponding to these
     indices, and display it. 

"""

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber")

voxgrid = ecto_pcl.VoxelGrid("VoxelGrid", leaf_size=0.01)

normals = ecto_pcl.NormalEstimation("Normals", k_search=0, radius_search=0.02, spatial_locator=ecto_pcl.KDTREE_FLANN)

segment = ecto_pcl.SACSegmentationFromNormals("PlanarSegmentation", model_type=ecto_pcl.SACMODEL_NORMAL_PLANE, eps_angle=0.09, distance_threshold=0.1)

project = ecto_pcl.ProjectInliers("ProjectInliers", model_type=ecto_pcl.SACMODEL_NORMAL_PLANE)

conhull = ecto_pcl.ConvexHull("ConvexHull")

prism = ecto_pcl.ExtractPolygonalPrismData("ExtractPrism", height_min=0.01, height_max=0.2)

extract = ecto_pcl.ExtractIndices("Extract", negative=False)

viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")

plasm.connect(grabber[:] >> voxgrid[:],
              # downsample, compute normals, do segmentation
              voxgrid[:] >> normals[:],
              voxgrid[:] >> segment["input"],
              normals[:] >> segment["normals"],
              # project inliers, find convex hull
              voxgrid[:] >> project["input"],
              segment["model"] >> project["model"],
              project[:] >> conhull[:],
              # extract stuff on table from original high-res cloud
              grabber[:] >> prism["input"],
              conhull[:] >> prism["planar_hull"],
              prism[:] >> extract["indices"],
              grabber[:] >> extract["input"],
              extract[:] >> viewer[:])

ecto.view_plasm(plasm)
sched = ecto.schedulers.Threadpool(plasm)
sched.execute()


