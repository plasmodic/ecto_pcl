#include "ecto_pcl.hpp"
#include <pcl/segmentation/extract_polygonal_prism_data.h>

struct ExtractPolygonalPrismData
{
  static void declare_params(ecto::tendrils& params)
  {
    pcl::ExtractPolygonalPrismData<cloud_t::PointType> default_;
    double height_min, height_max;
    default_.getHeightLimits (height_min, height_max);
    params.declare<double> ("height_min", "Minimum allowable height limits for the model.", height_min);
    params.declare<double> ("height_max", "Maximum allowable height limits for the model.", height_max);
    // todo: any tf, indices support?
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to segment.");
    inputs.declare<cloud_t::ConstPtr> ("planar_hull", "Planar hull to use.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
  }

  ExtractPolygonalPrismData() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set parameters
    double height_min = params.get<double> ("height_min");
    double height_max = params.get<double> ("height_max");
    impl_.setHeightLimits (height_min, height_max);

    //set in/out.
    input_ = inputs.at("input");
    hull_ = inputs.at("planar_hull");
    inliers_ = outputs.at("inliers");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    indices_t::Ptr inliers ( new indices_t() );

    // copy header
    inliers->header = (*input_)->header;
    
    impl_.setInputCloud (*input_);
    impl_.setInputPlanarHull (*hull_);

    impl_.segment (*inliers);

    //set the output.
    *inliers_ = inliers;
    return 0;
  }
  pcl::ExtractPolygonalPrismData<cloud_t::PointType> impl_;
  ecto::spore< cloud_t::ConstPtr > input_;
  ecto::spore< cloud_t::ConstPtr > hull_;
  ecto::spore< indices_t::ConstPtr > inliers_;

};

ECTO_CELL(ecto_pcl, ExtractPolygonalPrismData, "ExtractPolygonalPrismData", "Uses a set of point indices that respresent a planar model, and together with a given height, generates a 3D polygonal prism.");

