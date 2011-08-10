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
  pcl::io::loadPCDFile("cloud_binary.pcd", *cloud);
  pcl::io::savePCDFileASCII("cloud_ascii_wrote.pcd", *cloud);
  pcl::io::loadPCDFile("cloud_ascii_wrote.pcd", *cloud2);
  std::cout << "precision: " << std::numeric_limits<float>::digits10 << "\n";
  std::cout.precision(std::numeric_limits<float>::digits10);
  std::cout << std::endl;
  for (int i = 0, end = cloud->size(); i != end; i++)
  {
    pcl::PointXYZRGB pA, pB;
    pA = (*cloud)[i];
    pB = (*cloud2)[i];
    for (int j = 0; j < 4; j++)
    {
      if (pA.data[j] != pB.data[j] && std::isfinite(pA.data[j]))
      {
        std::cout << "diff: i=" << i << " j=" << j << " a=" << pA.data[j] << " b=" << pB.data[j] << "\n";
        return -1;
      }

    }
  }
  return 0;
}
