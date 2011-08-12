#!/usr/bin/env python

"""
This example shows how to extract points corresponding to objects on a table. 

  1) The example downsamples using a VoxelGrid before estimating 
     normals for the downsampled cloud. 
  2) These normals are then used for segmentation using RANSAC. 
  3) Segmentation produces a planar model to which all inliers are 
     projected so that a 2D convex hull can be created. 
  4) We then extract the indices of all points that are above the 
     plane formed by the convex hull. 
  5) Finally, we extract the point cloud corresponding to these
     indices, and display it. 

"""

import ecto, ecto_pcl, ecto_pcl_ros
import ecto_ros, ecto_sensor_msgs
import sys

PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2
PointCloudPub = ecto_sensor_msgs.Publisher_PointCloud2

def do_ecto():
    plasm = ecto.Plasm()
    
    sub = PointCloudSub("Cloud Subscriber", topic_name='/camera/rgb/points')
    
    grabber = ecto_pcl_ros.Message2PointCloud("Message2Cloud", format=ecto_pcl.XYZRGB)
    
    voxgrid = ecto_pcl.VoxelGrid("VoxelGrid", leaf_size=0.01)

    normals = ecto_pcl.NormalEstimation("Normals", k_search=0, radius_search=0.02)
    
    segment = ecto_pcl.SACSegmentationFromNormals("PlanarSegmentation", model_type=ecto_pcl.SACMODEL_NORMAL_PLANE, eps_angle=0.09, distance_threshold=0.1)
    
    project = ecto_pcl.ProjectInliers("ProjectInliers", model_type=ecto_pcl.SACMODEL_NORMAL_PLANE)
    
    conhull = ecto_pcl.ConvexHull("ConvexHull")
    
    prism = ecto_pcl.ExtractPolygonalPrismData("ExtractPrism", height_min=0.01, height_max=0.2)
    
    extract = ecto_pcl.ExtractIndices("Extract", negative=False)
    
    pcl2msg = ecto_pcl_ros.PointCloud2Message("Cloud2Message")

    pub = PointCloudPub("Cloud Publisher", topic_name='/ecto_pcl/tabletop')
    
    plasm.connect(  sub['output'] >> grabber[:],
                    grabber[:] >> voxgrid[:],
                    voxgrid[:] >> normals[:],
                    voxgrid[:] >> segment["input"],
                    normals[:] >> segment["normals"],
                    # project inliers, find convex hull
                    voxgrid[:] >> project["input"],
                    segment["model"] >> project["model"],
                    project[:] >> conhull[:],
                    # extract stuff on table from original high-res cloud
                    grabber[:] >> prism["input"],
                    conhull[:] >> prism["planar_hull"],
                    prism[:] >> extract["indices"],
                    grabber[:] >> extract["input"],
                    #convert output to ros msg and pub.
                    extract[:] >> pcl2msg[:],
                    pcl2msg[:] >> pub[:]
                  )
    
    sched = ecto.schedulers.Threadpool(plasm)
    #sched.execute()
    sched.execute_async()

    from IPython.Shell import IPShellEmbed
    ipshell = IPShellEmbed()
    ipshell()

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "table_top_segmentation")
    do_ecto()

