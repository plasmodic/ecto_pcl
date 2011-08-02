#!/usr/bin/env python

"""
This example shows how to capture a Point Cloud from 
a ROS openni_camera node and write it to a PCD file.
"""

import ecto, ecto_pcl, ecto_pcl_ros
import ecto_ros, ecto_sensor_msgs
import sys

PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2

def do_ecto():
    plasm = ecto.Plasm()

    sub = PointCloudSub("Subscriber", topic_name='/camera/rgb/points')
    
    grabber = ecto_pcl_ros.Message2PointCloud("Message2Cloud", format=ecto_pcl.XYZRGB)

    writer = ecto_pcl.PCDWriter("Writer", filename="test.pcd")

    plasm.connect(sub['output'] >> grabber[:],
                  grabber[:] >> writer[:])

    sched = ecto.schedulers.Threadpool(plasm)

    sched.execute()

if __name__=="__main__":
    ecto_ros.init(sys.argv, "pcd_writer")
    do_ecto()

