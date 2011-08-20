#!/usr/bin/env python

"""
This sample shows how to use the cloud viewer with a ROS-based plasm.
"""

import ecto, ecto_pcl
from ecto_openni import Capture

plasm = ecto.Plasm()

capture = Capture('device')
verter = ecto_pcl.NiConverter('verter')
viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

plasm.connect(capture[:] >> verter[:],
              verter[:] >> viewer[:])

if __name__ == "__main__":
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

