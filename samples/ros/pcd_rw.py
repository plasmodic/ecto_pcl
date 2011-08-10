#!/usr/bin/env python

"""
This example shows how to capture a Point Cloud from 
a ROS openni_camera node and write it to a PCD file.
"""
import ecto_ros
import ecto_pcl_ros
import ecto, ecto_pcl

def do_ecto():
    plasm = ecto.Plasm()

    writer = ecto_pcl.PCDWriter("Writer", filename_format="ascii_%04d.pcd", binary=False)
    reader = ecto_pcl.PCDReader("Reader", filename="cb.pcd")

    plasm.connect(reader[:] >> writer[:],
                  )

    sched = ecto.schedulers.Threadpool(plasm)

    sched.execute(niter=1)

if __name__=="__main__":
    do_ecto()

