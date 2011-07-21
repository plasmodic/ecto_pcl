/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ecto_pcl.hpp"
#include <pcl/segmentation/sac_segmentation.h>

#define DECLARESACSEGMENTATION(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::SACSegmentation< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARESACSEGMENTATION, ~, ECTO_XYZ_POINT_TYPES) > segmentation_variant_t;

/* no Segmentation.cpp -- these all derive from PCLBase */

struct SACSegmentation
{
  struct SegmentationParams {
    int model;
    int method;
    double eps_angle;
    double distance_threshold;
    int max_iterations;
    bool optimize_coefficients;
    double probability;
    double radius_min;
    double radius_max;
  };

  struct make_segmentation_variant : boost::static_visitor<segmentation_variant_t>
  {
    template <typename CloudType >
    segmentation_variant_t operator()(const CloudType& p) const
    {
      return segmentation_variant_t( typename pcl::SACSegmentation<typename CloudType::element_type::PointType>() );
    }
  };

  struct segmentation_configurator : boost::static_visitor<void>
  { 
    SegmentationParams& sp;
    segmentation_configurator(SegmentationParams& sp_) : sp(sp_) {}

    template <typename T>
    void operator()(T& impl_) const 
    { 
      impl_.setModelType(sp.model);
      impl_.setMethodType(sp.method);
      impl_.setEpsAngle(sp.eps_angle);
      impl_.setDistanceThreshold(sp.distance_threshold);
      impl_.setMaxIterations(sp.max_iterations);
      impl_.setOptimizeCoefficients(sp.optimize_coefficients);
      impl_.setProbability(sp.probability);
      impl_.setRadiusLimits (sp.radius_min, sp.radius_max);
    } 
  };

  static void declare_params(ecto::tendrils& params)
  {
    pcl::SACSegmentation<pcl::PointXYZ> default_;
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
    inputs.declare<PointCloud> ("input", "The cloud to segment.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
    outputs.declare<model_t::ConstPtr> ("model", "Model found during segmentation.");
  }

  SACSegmentation() : configured_(false) {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    input_ = inputs.at("input");
    inliers_ = outputs.at("inliers");
    model_ = outputs.at("model");

    SegmentationParams sp;
    sp.model = params.get<int> ("model_type");
    sp.method = params.get<int> ("method");
    sp.eps_angle = params.get<double> ("eps_angle");
    sp.distance_threshold = params.get<double> ("distance_threshold");
    sp.max_iterations = params.get<int> ("max_iterations");
    sp.optimize_coefficients = params.get<bool> ("optimize_coefficients");
    sp.probability = params.get<double> ("probability");
    sp.radius_min = params.get<double> ("radius_min");
    sp.radius_max = params.get<double> ("radius_max");
    
    xyz_cloud_variant_t cv = input_->make_variant();
    if(!configured_){
      impl_ = boost::apply_visitor(make_segmentation_variant(), cv);
      boost::apply_visitor(segmentation_configurator(sp), impl_);
      configured_ = true;
    }
  }

  /* dispatch to handle process */
  struct segmentation_dispatch : boost::static_visitor< std::pair<indices_t::ConstPtr, model_t::ConstPtr> >
  {
    segmentation_dispatch() { }

    template <typename Segmentation, typename CloudType>
    std::pair<indices_t::ConstPtr, model_t::ConstPtr> operator()(Segmentation& f, CloudType& i) const
    {
      return impl(f, i, pcl_takes_point_trait<Segmentation, CloudType>());
    }

    template <typename Segmentation, typename CloudType>
    std::pair<indices_t::ConstPtr, model_t::ConstPtr> impl(Segmentation& f, CloudType& i, boost::true_type) const
    {
      indices_t::Ptr inliers ( new indices_t() );
      model_t::Ptr model ( new model_t() );
      f.setInputCloud(i);
      f.segment(*inliers, *model);
      return std::pair<indices_t::ConstPtr, model_t::ConstPtr>(inliers, model);
    }

    template <typename Segmentation, typename CloudType>
    std::pair<indices_t::ConstPtr, model_t::ConstPtr> impl(Segmentation& s, CloudType& i, boost::false_type) const
    {
      throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
    }
  };


  int process(const tendrils& inputs, tendrils& outputs)
  {
    xyz_cloud_variant_t cvar = input_->make_variant();
    std::pair<indices_t::ConstPtr, model_t::ConstPtr> output = boost::apply_visitor(segmentation_dispatch(), impl_, cvar);
    //set the output.
    *inliers_ = output.first;
    *model_ = output.second;
    return 0;
  }

  bool configured_;
  ecto::spore<PointCloud> input_;
  ecto::spore<indices_t::ConstPtr> inliers_;
  ecto::spore<model_t::ConstPtr> model_;
  segmentation_variant_t impl_;

};
/*
#define DECLARESACSEGMENTATIONFROMNORMALS(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::SACSegmentationFromNormals< BOOST_PP_TUPLE_ELEM(2, 0, ELEM), pcl::Normal >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARESACSEGMENTATIONFROMNORMALS, ~, ECTO_XYZ_POINT_TYPES) > segmentation_from_normals_variant_t;

struct SACSegmentationFromNormals
{
  static void declare_params(ecto::tendrils& params)
  {
    pcl::SACSegmentationFromNormals<::pcl::PointXYZ, ::pcl::Normal> default_;
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
    params.declare<double> ("normal_distance_weight", "Relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) betwen point normals and the plane normal.", default_.getNormalDistanceWeight());
    // todo: any tf, indices support?
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<PointCloud> ("input", "The cloud to segment.");
    inputs.declare<FeatureCloud> ("normals", "The input normals.");
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
*/
ECTO_CELL(ecto_pcl, SACSegmentation, "SACSegmentation", "Segmentation using Sample Consensus.");
//ECTO_CELL(ecto_pcl, SACSegmentationFromNormals, "SACSegmentationFromNormals", "Segmentation using Sample Consensus from Normals.");

