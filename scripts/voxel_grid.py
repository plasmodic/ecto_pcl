#!/usr/bin/env python

"""
A VoxelGrid can be used to downsample very dense clouds.
This can be especially useful when estimating normals. 
"""

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber")

voxgrid = ecto_pcl.VoxelGrid("VGrid", leaf_size=0.05)

viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")

plasm.connect(grabber[:] >> voxgrid[:],
              voxgrid[:] >> viewer[:])

sched = ecto.schedulers.Threadpool(plasm)
sched.execute()


