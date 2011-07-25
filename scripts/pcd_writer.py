#!/usr/bin/env python

"""
This example shows how to capture a Kinect 
point cloud and write it to a PCD file.
"""

import ecto, ecto_pcl

plasm = ecto.Plasm()

grabber = ecto_pcl.KinectGrabber("Grabber")

writer = ecto_pcl.PCDWriter("Writer", filename="test.pcd")

plasm.connect(grabber[:] >> writer[:])

sched = ecto.schedulers.Threadpool(plasm)

if __name__=="__main__":
    sched.execute()


