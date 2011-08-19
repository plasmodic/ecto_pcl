#!/usr/bin/env python

"""
This example shows how to capture a Point Cloud from
a ROS openni_camera node and write it to a PCD file.
"""
import sys, ecto, ecto_ros, ecto_pcl_ros, ecto_sensor_msgs, ecto_pcl

plasm = ecto.Plasm()

sub = ecto_sensor_msgs.Subscriber_PointCloud2("Subscriber",
                                              topic_name='/camera/depth_registered/points')
msg2pc = ecto_pcl_ros.Message2PointCloud("Converter",
                                         format=ecto_pcl.XYZRGB)
writer = ecto_pcl.PCDWriter("Writer",
                            filename_format="ecto_cloud_binary_%04d.pcd", binary=True)

plasm.connect(sub[:] >> msg2pc[:],
              msg2pc[:] >> writer[:])

if __name__=="__main__":
    ecto_ros.init(sys.argv, "pcd_capture")
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=1)
