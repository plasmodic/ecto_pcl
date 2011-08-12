#!/usr/bin/env python

"""
MergeClouds allows you to merge 2 clouds together!
"""

import ecto, ecto_pcl, ecto_pcl_ros
import ecto_ros, ecto_sensor_msgs
import sys

PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2
PointCloudPub = ecto_sensor_msgs.Publisher_PointCloud2

def do_ecto():
    plasm = ecto.Plasm()

    sub = PointCloudSub("Subscriber", topic_name='/camera/rgb/points')
    
    grabber = ecto_pcl_ros.Message2PointCloud("Message2Cloud", format=ecto_pcl.XYZRGB)

    p1 = ecto_pcl.PassThrough("Pass", filter_field_name="z", filter_limit_max=2.0)
    p2 = ecto_pcl.PassThrough("Pass2", filter_field_name="z", filter_limit_min=3.0, filter_limit_max=8.0)

    voxgrid = ecto_pcl.VoxelGrid("VGrid", leaf_size=0.1)
    merge = ecto_pcl.MergeClouds("Merge")

    pcl2msg = ecto_pcl_ros.PointCloud2Message("Cloud2Message")

    pub = PointCloudPub("Cloud Publisher", topic_name='/ecto_pcl/sample_output')    

    plasm.connect(sub['output'] >> grabber[:],
                  grabber[:] >> p1[:],
                  grabber[:] >> p2[:],
                  p1[:] >> voxgrid[:],
                  voxgrid[:] >> merge["input"],
                  p2[:] >> merge["input2"],
                  merge[:] >> pcl2msg[:],
                  pcl2msg[:] >> pub[:])

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "merge_clouds")
    do_ecto()

