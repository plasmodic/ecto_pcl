#include "ecto_pcl.hpp"
#include <pcl/filters/voxel_grid.h>

struct VoxelGrid
{
  static void declare_params(ecto::tendrils& params)
  {
    // filter params
    pcl::VoxelGrid<cloud_t::PointType> default_;
    params.declare<std::string> ("filter_field_name", "The name of the field to use for filtering.", "");
    double filter_limit_min, filter_limit_max;
    default_.getFilterLimits(filter_limit_min, filter_limit_max);
    params.declare<double> ("filter_limit_min", "Minimum value for the filter.", filter_limit_min);
    params.declare<double> ("filter_limit_max", "Maximum value for the filter.", filter_limit_max);
    params.declare<bool> ("filter_limit_negative", "To negate the limits or not.", default_.getFilterLimitsNegative());

    // custom params
    params.declare<float> ("leaf_size", "The size of the leaf(meters), smaller means more points...", 0.05);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to filter");
    outputs.declare<cloud_t::ConstPtr> ("output", "Filtered cloud.");
  }

  VoxelGrid() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set filter details
    std::string filter_field_name = params.get<std::string> ("filter_field_name");
    impl_.setFilterFieldName(filter_field_name);
    double filter_limit_min = params.get<double> ("filter_limit_min");
    double filter_limit_max = params.get<double> ("filter_limit_max");
    impl_.setFilterLimits(filter_limit_min,filter_limit_max);
    bool filter_limit_negative = params.get<bool> ("filter_limit_negative");
    impl_.setFilterLimitsNegative(filter_limit_negative);

    // set the voxel grid size
    float leaf_size = params.get<float> ("leaf_size");
    impl_.setLeafSize(leaf_size, leaf_size, leaf_size);

    // set in/out
    input_ = inputs.at("input");
    output_ = outputs.at("output");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cloud_t::Ptr cloud ( new cloud_t() );
    // filter it.
    impl_.setInputCloud(*input_);
    impl_.filter(*cloud);
    // set the output.
    *output_ = cloud;
    return 0;
  }
  pcl::VoxelGrid<cloud_t::PointType> impl_;
  ecto::spore<cloud_t::ConstPtr> input_;
  ecto::spore<cloud_t::ConstPtr> output_;

};

ECTO_CELL(ecto_pcl, VoxelGrid, "VoxelGrid", "Voxel grid filter");


