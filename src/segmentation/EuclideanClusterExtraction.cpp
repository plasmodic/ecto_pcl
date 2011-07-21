#include "ecto_pcl.hpp"
#include <pcl/segmentation/extract_clusters.h>

struct EuclideanClusterExtraction
{
  typedef std::vector<indices_t> cluster_t;

  static void declare_params(ecto::tendrils& params)
  {
    pcl::EuclideanClusterExtraction<cloud_t::PointType> default_;
    params.declare<double> ("cluster_tolerance", "Spatial cluster tolerance as a measure in the L2 Euclidean space.", default_.getClusterTolerance());
    params.declare<int> ("min_cluster_size", "Minimum number of points that a cluster needs to contain in order to be considered valid.", default_.getMinClusterSize());
    params.declare<int> ("max_cluster_size", "Maximum number of points that a cluster needs to contain in order to be considered valid.", default_.getMaxClusterSize());
    params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to segment.");
    outputs.declare<cluster_t> ("output", "Clusters.");
  }

  EuclideanClusterExtraction() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set parameters
    double cluster_tolerance = params.get<double> ("cluster_tolerance"); 
    impl_.setClusterTolerance (cluster_tolerance);
    int min_cluster_size = params.get<int> ("min_cluster_size"); 
    impl_.setMinClusterSize (min_cluster_size);
    int max_cluster_size = params.get<int> ("max_cluster_size"); 
    impl_.setMaxClusterSize (max_cluster_size);
    int locator = params.get<int> ("spatial_locator");    
    impl_.setSpatialLocator (locator);

    // set in/out.
    input_ = inputs.at("input");
    output_ = outputs.at("output");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cluster_t clusters;

    impl_.setInputCloud (*input_);
    impl_.extract (clusters);

    //set the output.
    *output_ = clusters;
    return 0;
  }
  pcl::EuclideanClusterExtraction<cloud_t::PointType> impl_;
  ecto::spore< cloud_t::ConstPtr > input_;
  ecto::spore< cluster_t > output_;

};

ECTO_CELL(ecto_pcl, EuclideanClusterExtraction, "EuclideanClusterExtraction", "Segmentation for cluster extraction in a Euclidean sense.");

