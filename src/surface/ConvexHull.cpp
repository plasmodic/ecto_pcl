#include "ecto_pcl.hpp"
#include <pcl/surface/convex_hull.h>

struct ConvexHull
{ 
  static void declare_params(ecto::tendrils& params)
  {
    // no params
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to filter");
    outputs.declare<cloud_t::ConstPtr> ("output", "Feature cloud.");
  }

  ConvexHull() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set in/out
    input_ = inputs.at("input");
    output_ = outputs.at("output");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cloud_t::Ptr cloud ( new cloud_t() );
    // compute it.
    impl_.setInputCloud(*input_);
    impl_.reconstruct(*cloud);
    // set the output.
    *output_ = cloud;
    return 0;
  }
  pcl::ConvexHull<cloud_t::PointType> impl_;
  ecto::spore<cloud_t::ConstPtr> input_;
  ecto::spore<cloud_t::ConstPtr> output_;

};

ECTO_CELL(ecto_pcl, ConvexHull, "ConvexHull", "Using libqhull library.");

