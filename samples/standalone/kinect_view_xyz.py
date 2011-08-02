#!/usr/bin/env python

"""
This example shows how to capture a Kinect point cloud with
XYZ-only (no RGB information) and display it to a cloud viewer.
"""

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber", format=ecto_pcl.XYZ)

viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")

plasm.connect(grabber[:] >> viewer[:])

sched = ecto.schedulers.Threadpool(plasm)
sched.execute()


