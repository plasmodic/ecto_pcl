#!/usr/bin/env python

"""
A PassThrough filter can filter a point cloud on a single field. 
In this example we set the field to "z", and filter all points
with "z" > 2.0 (meters). One could also set a filter_limit_min.
"""

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber")

passthru = ecto_pcl.PassThrough("Pass", filter_field_name="z", filter_limit_max=2.0)

viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")

plasm.connect(grabber[:] >> passthru[:],
              passthru[:] >> viewer[:])

sched = ecto.schedulers.Threadpool(plasm)
sched.execute()


