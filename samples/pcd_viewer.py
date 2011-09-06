#!/usr/bin/env python

import ecto, ecto_pcl
import sys

plasm = ecto.Plasm()

reader = ecto_pcl.PCDReader("Reader", filename=sys.argv[1])
viewer = ecto_pcl.CloudViewer("viewer", window_name="PCD Viewer")

plasm.connect(reader[:] >> viewer[:])

if __name__=="__main__":
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=1)
