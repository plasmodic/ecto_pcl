#!/usr/bin/env python

import sys, ecto, ecto_pcl, ecto_ros, ecto_pcl_ros, ecto_sensor_msgs

plasm = ecto.Plasm()

sub = ecto_sensor_msgs.Subscriber_PointCloud2(topic_name='/camera/depth_registered/points')

grabber = ecto_pcl_ros.Message2PointCloud(format=ecto_pcl.XYZRGB)

pcl2msg = ecto_pcl_ros.PointCloud2Message()

pub = ecto_sensor_msgs.Publisher_PointCloud2(topic_name='/ecto_pcl/sample_output')

plasm.connect(sub['output'] >> grabber[:],
                  grabber[:] >> pcl2msg[:],
                  pcl2msg[:] >> pub[:])


if __name__ == "__main__":
    ecto_ros.init(sys.argv, "pass_it_on")
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()

