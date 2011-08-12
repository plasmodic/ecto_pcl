// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int
main(int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> CloudT;
  typedef CloudT::Ptr PtrT;

  PtrT cloud(new CloudT), cloud2(new CloudT);
  return 0;
}
