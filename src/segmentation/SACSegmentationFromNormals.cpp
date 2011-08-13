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
#include "pcl_cell_with_normals.hpp"
#include <pcl/segmentation/sac_segmentation.h>

struct SACSegmentationFromNormals
{
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
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
    outputs.declare<model_t::ConstPtr> ("model", "Model found during segmentation.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    model_type_ = params["model_type"];
    method_ = params["method"];
    eps_angle_ = params["eps_angle"];
    distance_threshold_ = params["distance_threshold"];
    max_iterations_ = params["max_iterations"];
    optimize_coefficients_ = params["optimize_coefficients"];
    probability_ = params["probability"];
    radius_min_ = params["radius_min"];
    radius_max_ = params["radius_max"];
    normal_distance_weight_ = params["normal_distance_weight"];

    inliers_ = outputs["inliers"];
    model_ = outputs["model"];
  }

  template <typename Point>
  int process(const tendrils& inputs, const tendrils& outputs, 
              boost::shared_ptr<const pcl::PointCloud<Point> >& input,
              boost::shared_ptr<const pcl::PointCloud<pcl::Normal> >& normals)
  {
    pcl::SACSegmentationFromNormals<Point,pcl::Normal> impl;
    indices_t::Ptr inliers ( new indices_t() );
    model_t::Ptr model ( new model_t() );

    impl.setModelType(*model_type_);
    impl.setMethodType(*method_);
    impl.setEpsAngle(*eps_angle_);
    impl.setDistanceThreshold(*distance_threshold_);
    impl.setMaxIterations(*max_iterations_);
    impl.setOptimizeCoefficients(*optimize_coefficients_);
    impl.setProbability(*probability_);
    impl.setRadiusLimits(*radius_min_, *radius_max_);

    impl.setInputNormals(normals);
    impl.setInputCloud(input);
    impl.segment(*inliers, *model);

    *model_ = model;
    *inliers_ = inliers;
    return ecto::OK;
  }

  ecto::spore<int> model_type_;
  ecto::spore<int> method_;
  ecto::spore<double> eps_angle_;
  ecto::spore<double> distance_threshold_;
  ecto::spore<int> max_iterations_;
  ecto::spore<bool> optimize_coefficients_;
  ecto::spore<double> probability_;
  ecto::spore<double> radius_min_;
  ecto::spore<double> radius_max_;
  ecto::spore<double> normal_distance_weight_;

  ecto::spore< indices_t::ConstPtr > inliers_;
  ecto::spore< model_t::ConstPtr > model_;
};

ECTO_CELL(ecto_pcl, ecto::pcl::PclCellWithNormals<SACSegmentationFromNormals>, "SACSegmentationFromNormals", "Segmentation using Sample Consensus from Normals.");

