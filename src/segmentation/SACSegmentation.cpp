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

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <pcl/segmentation/sac_segmentation.h>

namespace ecto {
  namespace pcl {

    struct SACSegmentation
    {
      static void declare_params(ecto::tendrils& params)
      {
        ::pcl::SACSegmentation< ::pcl::PointXYZ > default_;
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
        params.declare<double> ("axis_x", "X component of desired perpendicular vector for model.", 0.0);
        params.declare<double> ("axis_y", "Y component of desired perpendicular vector for model.", 0.0);
        params.declare<double> ("axis_z", "Z component of desired perpendicular vector for model.", 0.0);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        outputs.declare<Indices::ConstPtr> ("inliers", "Inliers of the model.");
        outputs.declare<ModelCoefficients::ConstPtr> ("model", "Model found during segmentation.");
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
        axis_x_ = params["axis_x"];
        axis_y_ = params["axis_y"];
        axis_z_ = params["axis_z"];

        inliers_ = outputs["inliers"];
        model_ = outputs["model"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs, 
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::SACSegmentation<Point> impl;
        Indices::Ptr inliers ( new Indices() );
        ModelCoefficients::Ptr model ( new ModelCoefficients() );

        impl.setModelType(*model_type_);
        impl.setMethodType(*method_);
        impl.setEpsAngle(*eps_angle_);
        impl.setDistanceThreshold(*distance_threshold_);
        impl.setMaxIterations(*max_iterations_);
        impl.setOptimizeCoefficients(*optimize_coefficients_);
        impl.setProbability(*probability_);
        impl.setRadiusLimits(*radius_min_, *radius_max_);
        impl.setAxis(Eigen::Vector3f(*axis_x_, *axis_y_, *axis_z_));
        impl.segment (*inliers, *model);
        impl.setInputCloud(input);

        *model_ = model;
        *inliers_ = inliers;
        return OK;
      }

      spore<int> model_type_;
      spore<int> method_;
      spore<double> eps_angle_;
      spore<double> distance_threshold_;
      spore<int> max_iterations_;
      spore<bool> optimize_coefficients_;
      spore<double> probability_;
      spore<double> radius_min_;
      spore<double> radius_max_;
      spore<double> axis_x_, axis_y_, axis_z_;
      spore<Indices::ConstPtr> inliers_;
      spore<ModelCoefficients::ConstPtr> model_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::SACSegmentation>,
          "SACSegmentation", "Segmentation using Sample Consensus.");

