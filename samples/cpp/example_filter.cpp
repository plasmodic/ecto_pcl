/* 
 * Example of how to use ecto_pcl with custom code.
 */

#include "ecto_pcl.hpp"
#include "pcl_cell.hpp"

// other pcl includes!

struct MyCell
{
  static void declare_params(ecto::tendrils& params)
  {
    // put declarations here as usual!
    params.declare<int> ("a_param", "Description of params.", 0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    /* 
     * A single "ecto::pcl::PointCloud" input is already defined... any others should be here
     * If you need 2 input PointClouds, use the PclCellDualInputs
     */

    // Most cells will output a PointCloud
    outputs.declare<PointCloud> ("output", "Cloud after my stufz has run.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    // Store params in spores
    my_param_ = params["a_param"];

    // Same for inputs/outputs
    output_ = outputs["output"];
  }
  
  template <typename Point>
  int process(const tendrils& inputs, const tendrils& outputs, 
              boost::shared_ptr<const pcl::PointCloud<Point> >& input)
  {
    // cloud to store our output in
    pcl::PointCloud<Point> cloud;

    // do something with our params/clouds
    cloud = input;

    // Need to a PointCloud
    *output_ = xyz_cloud_variant_t(cloud.makeShared());
    return ecto::OK;
  }

  // Store params/inputs/outputs in spores
  ecto::spore<int> my_param_;
  ecto::spore<PointCloud> output_;
};

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<MyCell>, "MyCell", "Useless Example");

