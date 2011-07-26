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

#define DECLARESACSEGMENTATIONFROMNORMALS(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::SACSegmentationFromNormals< BOOST_PP_TUPLE_ELEM(2, 0, ELEM), pcl::Normal >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARESACSEGMENTATIONFROMNORMALS, ~, ECTO_XYZ_POINT_TYPES) > segmentation_variant_t;

#include <segmentation/Segmentation.hpp>
#include <boost/variant/get.hpp>

struct SACSegmentationFromNormals
{
  template <typename Point>
  struct segmentation {
    typedef typename ::pcl::SACSegmentationFromNormals<Point, ::pcl::Normal> type;
  };

  static void declare_params(ecto::tendrils& params)
  {
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> default_;
    params.declare<int> ("model_type", "Type of model to use.", default_.getModelType());
    params.declare<int> ("method", "Type of sample consensus method to use.", default_.getMethodType());
    params.declare<double> ("eps_angle", "Angle epsilon (delta) threshold.", default_.getEpsAngle());
    params.declare<double> ("distance_threshold", "Doistance to model threshold.", default_.getDistanceThreshold());
    params.declare<int> ("max_iterations", "Maximum number of iterations before giving up.", default_.getMaxIterations());
    params.declare<bool> ("optimize_coefficients", "True if a coefficient refinement is required.", default_.getOptimizeCoefficients());
    params.declare<double> ("probability", "Probability of choosing at least one sample free from outliers.", default_.getProbability());
    double t_min, t_max;
    default_.getRadiusLimits (t_min, t_max);
    params.declare<double> ("radius_min", "Minimum allowable radius limits for the model.", t_min);
    params.declare<double> ("radius_max", "Maximum allowable radius limits for the model.", t_max);    
    params.declare<double> ("normal_distance_weight", "Relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) betwen point normals and the plane normal.", default_.getNormalDistanceWeight());
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare<FeatureCloud> ("normals", "The input normals.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
    outputs.declare<model_t::ConstPtr> ("model", "Model found during segmentation.");
  }

  template <typename Point>
  void configure(pcl::SACSegmentation<Point>& impl_)
  {
    impl_.setModelType(model);
    impl_.setMethodType(method);
    impl_.setEpsAngle(eps_angle);
    impl_.setDistanceThreshold(distance_threshold);
    impl_.setMaxIterations(max_iterations);
    impl_.setOptimizeCoefficients(optimize_coefficients);
    impl_.setProbability(probability);
    impl_.setRadiusLimits(radius_min, radius_max);
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    normals_ = inputs.at("normals");
    inliers_ = outputs.at("inliers");
    model_ = outputs.at("model");

    model = params.get<int> ("model_type");
    method = params.get<int> ("method");
    eps_angle = params.get<double> ("eps_angle");
    distance_threshold = params.get<double> ("distance_threshold");
    max_iterations = params.get<int> ("max_iterations");
    optimize_coefficients = params.get<bool> ("optimize_coefficients");
    probability = params.get<double> ("probability");
    radius_min = params.get<double> ("radius_min");
    radius_max = params.get<double> ("radius_max");
    normal_distance_weight = params.get<double> ("normal_distance_weight");
  }

  template <typename Point>
  int process(pcl::SACSegmentationFromNormals<Point, ::pcl::Normal>& impl_) {
    indices_t::Ptr inliers ( new indices_t() );
    model_t::Ptr model ( new model_t() );

    // copy header
    //inliers->header = model->header = (*input_)->header;

    feature_cloud_variant_t cv = normals_->make_variant();
    try{
      CloudNORMAL::ConstPtr c = boost::get<CloudNORMAL::ConstPtr>(cv);
      if(c)
        impl_.setInputNormals (c);
    }catch(boost::bad_get){
      throw std::runtime_error("SACSegmentation works only with pcl::Normal feature clouds!");
    }
    
    impl_.segment (*inliers, *model);

    *model_ = model;
    *inliers_ = inliers;

    return 0;
  }
  int process(const tendrils& inputs, tendrils& outputs) { return 0; }

  int model;
  int method;
  double eps_angle;
  double distance_threshold;
  int max_iterations;
  bool optimize_coefficients;
  double probability;
  double radius_min;
  double radius_max;
  double normal_distance_weight;

  ecto::spore< FeatureCloud > normals_;
  ecto::spore< indices_t::ConstPtr > inliers_;
  ecto::spore< model_t::ConstPtr > model_;

};

ECTO_CELL(ecto_pcl, ecto::pcl::SegmentationCell<SACSegmentationFromNormals>, "SACSegmentationFromNormals", "Segmentation using Sample Consensus from Normals.");

