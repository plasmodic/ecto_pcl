#!/usr/bin/env python

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber")

voxgrid = ecto_pcl.VoxelGrid("VoxelGrid", leaf_size=0.01)

normals = ecto_pcl.NormalEstimation("Normals", k_search=0, radius_search=0.02, spatial_locator=ecto_pcl.KDTREE_FLANN)

segment = ecto_pcl.SACSegmentationFromNormals("PlanarSegmentation", model=ecto_pcl.SACMODEL_NORMAL_PLANE, eps_angle=0.09, distance_threshold=0.1)

extract = ecto_pcl.ExtractIndices("Extract", negative=True)

viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")

plasm.connect(grabber[:] >> voxgrid[:],
              voxgrid[:] >> normals[:],
              voxgrid[:] >> segment["input"],
              voxgrid[:] >> extract["input"],
              normals[:] >> segment["normals"],
              segment["inliers"] >> extract["indices"],
              extract[:] >> viewer[:])

ecto.view_plasm(plasm)
sched = ecto.schedulers.Threadpool(plasm)
sched.execute(niter=100, nthreads=4)


