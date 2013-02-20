#!/usr/bin/env python

"""
This sample shows how to interact with ROS cloud subscribers and publishers.
"""

import sys, ecto, ecto_pcl, ecto_ros, ecto_pcl_ros, ecto_sensor_msgs

plasm = ecto.Plasm()

cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)

cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/sample_output')

plasm.connect(cloud_sub['output'] >> msg2cloud[:],
              msg2cloud[:] >> cloud2msg[:],
              cloud2msg[:] >> cloud_pub[:])


if __name__ == "__main__":
    from ecto.opts import doit
    doit(plasm, description='Read a pcd through ROS.')
