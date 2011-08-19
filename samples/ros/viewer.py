#!/usr/bin/env python

"""
This sample shows how to use the cloud viewer with a ROS-based plasm.
"""

import ecto, ecto_pcl, ecto_pcl_ros
import ecto_ros, ecto_sensor_msgs
import sys

PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2

plasm = ecto.Plasm()
sub = PointCloudSub("Subscriber", topic_name='/camera/depth_registered/points')
grabber = ecto_pcl_ros.Message2PointCloud("Message2Cloud", format=ecto_pcl.XYZRGB)
viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

plasm.connect(sub['output'] >> grabber[:],
              grabber[:] >> viewer[:])

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "passthrough")
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

