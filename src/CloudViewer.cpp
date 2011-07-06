#include "ecto_pcl.hpp"
#include <pcl/visualization/cloud_viewer.h>

struct CloudViewer
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string> ("window_name", "The window name", "cloud viewer");

  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to view");
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
    cloud_t::ConstPtr cloud = inputs.get<cloud_t::ConstPtr> ("input");
    if (cloud)
      viewer_->showCloud(cloud, "cloud");
    if (viewer_->wasStopped(10))
      outputs.get<bool> ("stop") = true;
    return 0;
  }
  boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;
};

ECTO_CELL(ecto_pcl, CloudViewer, "CloudViewer", "Viewer of clouds");

