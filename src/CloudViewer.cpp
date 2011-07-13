#include "ecto_pcl.hpp"
#include <pcl/visualization/cloud_viewer.h>

#include <boost/variant/get.hpp>

typedef boost::variant< boost::shared_ptr<pcl::visualization::CloudViewer> > cloud_viewer_variant_t;

struct CloudViewer
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string> ("window_name", "The window name", "cloud viewer");

  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<PointCloud> ("input", "The cloud to view");
    outputs.declare<bool> ("stop", "True if stop requested", false);
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    viewer_.reset(new pcl::visualization::CloudViewer(params.get<std::string> ("window_name")));
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    if (!viewer_)
      return 1;

    PointCloud cloud = inputs.get<PointCloud> ("input");
    cloud_variant_t cv = cloud.make_variant();
    try{
      PointCloudXYZRGB::ConstPtr c = boost::get<PointCloudXYZRGB::ConstPtr>(cv);
      if(c)
        viewer_->showCloud(c, "cloud");
    }catch(boost::bad_get){
      try{
        PointCloudXYZ::ConstPtr c = boost::get<PointCloudXYZ::ConstPtr>(cv);
        if(c)
          viewer_->showCloud(c, "cloud");
      }catch(boost::bad_get){
        throw std::runtime_error("CloudViewer supports only XYZ and XYZRGB point clouds!");
      }
    }
    if (viewer_->wasStopped(10))
      outputs.get<bool> ("stop") = true;
    return 0;
  }
  boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;
};

ECTO_CELL(ecto_pcl, CloudViewer, "CloudViewer", "Viewer of clouds");

