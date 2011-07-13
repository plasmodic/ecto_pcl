#include "ecto_pcl.hpp"
#include <pcl/segmentation/sac_segmentation.h>

struct SACSegmentation
{
  static void declare_params(ecto::tendrils& params)
  {
    pcl::SACSegmentation<cloud_t::PointType> default_;
    params.declare<int> ("model_type", "Type of model to use.", default_.getModelType());
    params.declare<int> ("method", "Type of sample consensus method to use.", default_.getMethodType());
    params.declare<double> ("eps_angle", "Angle epsilon (delta) threshold.", default_.getEpsAngle());
    params.declare<double> ("distance_threshold", "Doistance to model threshold.", default_.getDistanceThreshold());
    params.declare<int> ("max_iterations", "Maximum number of iterations before giving up.", default_.getMaxIterations());
    params.declare<bool> ("optimize_coefficients", "True if a coefficient refinement is required.", default_.getOptimizeCoefficients());
    params.declare<double> ("probability", "Probability of choosing at least one sample free from outliers.", default_.getProbability());
    double radius_min, radius_max;
    default_.getRadiusLimits (radius_min, radius_max);
    params.declare<double> ("radius_min", "Minimum allowable radius limits for the model.", radius_min);
    params.declare<double> ("radius_max", "Maximum allowable radius limits for the model.", radius_max);
    // todo: any tf, indices support?
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to segment.");
    //outputs.declare<cloud_t::ConstPtr> ("output", "Segmented output cloud.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
    outputs.declare<model_t::ConstPtr> ("model", "Model found during segmentation.");
  }

  SACSegmentation() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set parameters
    int model = params.get<int> ("model_type");
    impl_.setModelType(model);
    int method = params.get<int> ("method");
    impl_.setMethodType(method);
    double eps_angle = params.get<double> ("eps_angle");
    impl_.setEpsAngle(eps_angle);
    double distance_threshold = params.get<double> ("distance_threshold");
    impl_.setDistanceThreshold(distance_threshold);
    int max_iterations = params.get<int> ("max_iterations");
    impl_.setMaxIterations(max_iterations);
    bool optimize_coefficients = params.get<bool> ("optimize_coefficients");
    impl_.setOptimizeCoefficients(optimize_coefficients);
    double probability = params.get<double> ("probability");
    impl_.setProbability(probability);
    double radius_min = params.get<double> ("radius_min");
    double radius_max = params.get<double> ("radius_max");
    impl_.setRadiusLimits (radius_min, radius_max);

    //set in/out.
    input_ = inputs.at("input");
    inliers_ = outputs.at("inliers");
    model_ = outputs.at("model");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    indices_t::Ptr inliers ( new indices_t() );
    model_t::Ptr model ( new model_t() );

    // copy header
    inliers->header = model->header = (*input_)->header;
    
    impl_.setInputCloud (*input_);
    //impl_.setIndices (indices_ptr);

    impl_.segment (*inliers, *model);

    //set the output.
    *model_ = model;
    *inliers_ = inliers;
    return 0;
  }
  pcl::SACSegmentation<cloud_t::PointType> impl_;
  ecto::spore< cloud_t::ConstPtr > input_;
  ecto::spore< indices_t::ConstPtr > inliers_;
  ecto::spore< model_t::ConstPtr > model_;

};

struct SACSegmentationFromNormals
{
  static void declare_params(ecto::tendrils& params)
  {
    // segmentation params
    pcl::SACSegmentationFromNormals<cloud_t::PointType, normals_t::PointType> default_;
    params.declare<int> ("model_type", "Type of model to use.", default_.getModelType());
    params.declare<int> ("method", "Type of sample consensus method to use.", default_.getMethodType());
    params.declare<double> ("eps_angle", "Angle epsilon (delta) threshold.", default_.getEpsAngle());
    params.declare<double> ("distance_threshold", "Doistance to model threshold.", default_.getDistanceThreshold());
    params.declare<int> ("max_iterations", "Maximum number of iterations before giving up.", default_.getMaxIterations());
    params.declare<bool> ("optimize_coefficients", "True if a coefficient refinement is required.", default_.getOptimizeCoefficients());
    params.declare<double> ("probability", "Probability of choosing at least one sample free from outliers.", default_.getProbability());
    double radius_min, radius_max;
    default_.getRadiusLimits (radius_min, radius_max);
    params.declare<double> ("radius_min", "Minimum allowable radius limits for the model.", radius_min);
    params.declare<double> ("radius_max", "Maximum allowable radius limits for the model.", radius_max);

    // custom params
    params.declare<double> ("normal_distance_weight", "Relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) betwen point normals and the plane normal.", default_.getNormalDistanceWeight());

    // todo: any tf, indices support?
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to segment.");
    inputs.declare<normals_t::ConstPtr> ("normals", "The input normals.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
    outputs.declare<model_t::ConstPtr> ("model", "Model found during segmentation.");
  }

  SACSegmentationFromNormals() {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set parameters
    int model = params.get<int> ("model_type");
    impl_.setModelType(model);
    int method = params.get<int> ("method");
    impl_.setMethodType(method);
    double eps_angle = params.get<double> ("eps_angle");
    impl_.setEpsAngle(eps_angle);
    double distance_threshold = params.get<double> ("distance_threshold");
    impl_.setDistanceThreshold(distance_threshold);
    int max_iterations = params.get<int> ("max_iterations");
    impl_.setMaxIterations(max_iterations);
    bool optimize_coefficients = params.get<bool> ("optimize_coefficients");
    impl_.setOptimizeCoefficients(optimize_coefficients);
    double probability = params.get<double> ("probability");
    impl_.setProbability(probability);
    double radius_min = params.get<double> ("radius_min");
    double radius_max = params.get<double> ("radius_max");
    impl_.setRadiusLimits (radius_min, radius_max);
    double normal_distance_weight = params.get<double> ("normal_distance_weight");
    impl_.setNormalDistanceWeight(normal_distance_weight);

    //set in/out.
    input_ = inputs.at("input");
    normals_ = inputs.at("normals");
    inliers_ = outputs.at("inliers");
    model_ = outputs.at("model");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    indices_t::Ptr inliers ( new indices_t() );
    model_t::Ptr model ( new model_t() );

    // copy header
    inliers->header = model->header = (*input_)->header;
    
    impl_.setInputCloud (*input_);
    impl_.setInputNormals (*normals_);
    //impl_.setIndices (indices_ptr);

    impl_.segment (*inliers, *model);

    //set the output.
    *model_ = model;
    *inliers_ = inliers;
    return 0;
  }
  pcl::SACSegmentationFromNormals<cloud_t::PointType, normals_t::PointType> impl_;
  ecto::spore< cloud_t::ConstPtr > input_;
  ecto::spore< normals_t::ConstPtr > normals_;
  ecto::spore< indices_t::ConstPtr > inliers_;
  ecto::spore< model_t::ConstPtr > model_;

};

ECTO_CELL(ecto_pcl, SACSegmentation, "SACSegmentation", "Segmentation using Sample Consensus.");
ECTO_CELL(ecto_pcl, SACSegmentationFromNormals, "SACSegmentationFromNormals", "Segmentation using Sample Consensus from Normals.");

