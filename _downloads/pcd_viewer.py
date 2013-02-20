#!/usr/bin/env python

import ecto, ecto_pcl
import sys
import time
import os

plasm = ecto.Plasm()
pcdfile = os.path.join(os.path.dirname(__file__),
                       'cloud.pcd')

if len(sys.argv) > 1:
    pcdfile = sys.argv[1]

reader = ecto_pcl.PCDReader("Reader",
                            filename=pcdfile)

viewer = ecto_pcl.CloudViewer("viewer",
                              window_name="PCD Viewer")

plasm.connect(reader[:] >> viewer[:])

if __name__=="__main__":
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(niter=1)
    #sleep 2 seconds and exit.
    time.sleep(2)
