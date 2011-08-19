#!/usr/bin/env python

"""
Read a point cloud from a pcd file and display it repeatedly.
"""
import sys, ecto, ecto_pcl

plasm = ecto.Plasm()

reader = ecto_pcl.PCDReader("Reader", filename=sys.argv[1])
viewer = ecto_pcl.CloudViewer()

plasm.connect(reader[:] >> viewer[:])

if __name__=="__main__":
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=50)


