#!/usr/bin/env python

"""
This example shows how to capture a Kinect 
point cloud and write it to a PCD file. 
"""

import ecto, ecto_pcl

def do_ecto():
    plasm = ecto.Plasm()

    grabber = ecto_pcl.KinectGrabber("Grabber")

    writer = ecto_pcl.PCDWriter("Writer", filename="test.pcd")

    plasm.connect(grabber[:] >> writer[:])

    sched = ecto.schedulers.Threadpool(plasm)

    sched.execute()

if __name__=="__main__":
    do_ecto()

