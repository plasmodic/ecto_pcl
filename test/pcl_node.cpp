#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>

#include <boost/format.hpp>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  static int count = 0;
  typedef pcl::PointCloud<pcl::PointXYZRGB> CloudT;
  typedef CloudT::Ptr PtrT;
  PtrT cloud(new CloudT);
  pcl::fromROSMsg(*input, *cloud);
  pcl::io::savePCDFileASCII(boost::str(boost::format("%s%04d.pcd")%"cloud_"%count++), *cloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
