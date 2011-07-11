#include "ecto_pcl.hpp"
#include <pcl/features/normal_3d.h>

struct NormalEstimation
{ 
  static void declare_params(ecto::tendrils& params)
  {
    // filter params
    params.declare<int> ("k_search", "The number of k nearest neighbors to use for feature estimation.", 0);
    params.declare<double> ("radius_search", "The sphere radius to use for determining the nearest neighbors used for feature estimation.", 0);
    params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to filter");
    outputs.declare<normals_t::ConstPtr> ("output", "Feature cloud.");
  }

  NormalEstimation() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set params
    int k = params.get<int> ("k_search");
    impl_.setKSearch(k);
    double radius = params.get<double> ("radius_search");
    impl_.setRadiusSearch(radius);

    // init spatial locator
    int locator = params.get<int> ("spatial_locator");    
    initTree (locator, tree_, k);
    impl_.setSearchMethod (tree_);

    // set in/out
    input_ = inputs.at("input");
    output_ = outputs.at("output");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    normals_t::Ptr cloud ( new normals_t() );
    // compute it.
    impl_.setInputCloud(*input_);
    //impl_.setIndices (indices);
    //impl_.setSearchSurface (surface);
    impl_.compute(*cloud);
    //cloud.header = input_->header;
    // set the output.
    *output_ = cloud;
    return 0;
  }
  pcl::NormalEstimation<cloud_t::PointType, pcl::Normal> impl_;
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree_;
  ecto::spore<cloud_t::ConstPtr> input_;
  ecto::spore<normals_t::ConstPtr> output_;

};

ECTO_CELL(ecto_pcl, NormalEstimation, "NormalEstimation", "Normal estimation");

