#!/usr/bin/env python

"""
This sample shows how to use the ecto_openni capture and cloud viewer.
"""

import ecto, ecto_pcl, ecto_openni

plasm = ecto.Plasm()

device = ecto_openni.Capture('device')
cloud_generator = ecto_pcl.NiConverter('cloud_generator')
viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

plasm.connect(device[:] >> cloud_generator[:],
              cloud_generator[:] >> viewer[:])

if __name__ == "__main__":
    from ecto.opts import doit
    doit(plasm, description='View the current point cloud.')
