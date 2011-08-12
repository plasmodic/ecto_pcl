#!/usr/bin/env python

"""
A PassThrough filter can filter a point cloud on a single field. 
In this example we set the field to "z", and filter all points
with "z" > 2.0 (meters). One could also set a filter_limit_min.
"""

import ecto, ecto_pcl, ecto_pcl_ros
import ecto_ros, ecto_sensor_msgs
import sys

PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2
PointCloudPub = ecto_sensor_msgs.Publisher_PointCloud2

def do_ecto():
    plasm = ecto.Plasm()

    sub = PointCloudSub("Subscriber", topic_name='/camera/depth_registered/points')
    
    grabber = ecto_pcl_ros.Message2PointCloud("Message2Cloud", format=ecto_pcl.XYZRGB)

    passthru = ecto_pcl.PassThrough("Pass", filter_field_name="z", filter_limit_max=2.0)

    pcl2msg = ecto_pcl_ros.PointCloud2Message("Cloud2Message", )

    pub = PointCloudPub("Cloud Publisher", topic_name='/ecto_pcl/sample_output')    

    plasm.connect(sub['output'] >> grabber[:],
                  grabber[:] >> passthru[:],
                  passthru[:] >> pcl2msg[:],
                  pcl2msg[:] >> pub[:])

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "passthrough")
    do_ecto()

