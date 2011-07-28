#!/usr/bin/env python

"""
This example shows how to capture a Kinect point 
cloud and display it to a cloud viewer.
"""

import ecto, ecto_pcl, ecto_pcl_ros
#PKG = 'ecto_ros' # this package name
#import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs
import sys
PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2
PointCloudPub = ecto_sensor_msgs.Publisher_PointCloud2
def do_ecto():
    plasm = ecto.Plasm()
    sub = PointCloudSub("Cloud Subscriber",topic_name='/camera/rgb/points')
    msg2pcl = ecto_pcl_ros.Message2PointCloud(format=ecto_pcl.XYZRGB)
    pcl2msg = ecto_pcl_ros.PointCloud2Message()
    voxgrid = ecto_pcl.VoxelGrid("VGrid", leaf_size=0.05)
    pub = PointCloudPub("Cloud Publisher", topic_name='/ecto_pcl/voxel')
    #viewer = ecto_pcl.CloudViewer("Viewer", window_name="Clouds!")
    plasm.connect(sub['output'] >> msg2pcl[:],
                  msg2pcl[:] >> voxgrid[:],
                  voxgrid[:] >> pcl2msg[:],
                  pcl2msg[:] >> pub[:]
                  #msg2pcl[:] >> viewer[:]
                  )
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute_async()

    from IPython.Shell import IPShellEmbed
    ipshell = IPShellEmbed()
    ipshell()
if __name__ == "__main__":
    ecto_ros.init(sys.argv,"image_node")
    do_ecto()
