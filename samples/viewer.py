#!/usr/bin/env python

"""
This sample shows how to use the cloud viewer with ecto_openni capture.
"""

import ecto, ecto_pcl, ecto_openni

plasm = ecto.Plasm()

capture = ecto_openni.Capture('device')
verter = ecto_pcl.NiConverter('verter')
viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

plasm.connect(capture[:] >> verter[:],
              verter[:] >> viewer[:])

if __name__ == "__main__":
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

