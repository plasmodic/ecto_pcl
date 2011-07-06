#include "ecto_pcl.hpp"
#include <pcl/filters/voxel_grid.h>

struct VoxelGrid
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<float> ("leaf_size", "The size of the leaf(meters), smaller means more points...", 0.05);

  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to filter");
    outputs.declare<cloud_t::ConstPtr> ("output", "Filtered cloud.");

  }
  VoxelGrid() 
    : cloud_out_(new cloud_t)
  { }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //set the voxel grid size
    float leaf_size = params.get<float> ("leaf_size");
    voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    //grab the input cloud
    cloud_t::ConstPtr cloud = inputs.get<cloud_t::ConstPtr> ("input");
    //voxel grid filter it.
    voxel_grid_.setInputCloud(cloud);
    voxel_grid_.filter(*cloud_out_);
    //set the output to the voxelized cloud.
    outputs.get<cloud_t::ConstPtr> ("output") = cloud_out_;
    return 0;
  }
  pcl::VoxelGrid<cloud_t::PointType> voxel_grid_;
  cloud_t::Ptr cloud_out_;

};

ECTO_CELL(ecto_pcl, VoxelGrid, "VoxelGrid", "Voxel grid filter");


