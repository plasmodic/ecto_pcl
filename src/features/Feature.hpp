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

#pragma once

#include <pcl/kdtree/kdtree.h>

namespace ecto {
  namespace pcl {

    template <typename FeatureEstimatorType>
    struct FeatureCell
    {
      FeatureEstimatorType custom;

      struct FeatureParams {
        int k;
        double radius;
        int locator;
      };
      FeatureParams fp;

      /* while output type is known, our input is a variant */
      template <template <class> class FeaturePolicy>
      struct make_feature_estimator_variant : boost::static_visitor<feature_estimator_variant_t>
      {
        template <typename CloudType >
        feature_estimator_variant_t operator()(const CloudType& p) const
        {
          return feature_estimator_variant_t(typename FeaturePolicy<typename CloudType::element_type::PointType>::type());
        }
      };

      struct feature_configurator : boost::static_visitor<void>
      { 
        FeatureParams& fp;
        FeatureEstimatorType& ft;
        feature_configurator(FeatureParams& fp_, FeatureEstimatorType& ft_) : fp(fp_), ft(ft_) {}

//typename FilterPolicy<typename CloudType::element_type::PointType>::type()
       // typename FilterType::template filter< ::pcl::PointXYZRGB>::type default_;
        template <typename T> //Point>
        //void operator()(typename FeatureEstimatorType::template feature_estimator<Point>::type& impl_) const 
        void operator()(T& impl_) const 
        { 
          impl_.setKSearch(fp.k);
          impl_.setRadiusSearch(fp.radius);
          typename T::KdTreePtr tree_;
          initTree (fp.locator, tree_, fp.k);
          impl_.setSearchMethod (tree_);
          ft.configure(impl_);
        } 
      };

      static void declare_params(ecto::tendrils& params)
      {
        typename FeatureEstimatorType::template feature_estimator< ::pcl::PointXYZ >::type default_;
        params.declare<int> ("k_search", "The number of k nearest neighbors to use for feature estimation.", default_.getKSearch());
        params.declare<double> ("radius_search", "The sphere radius to use for determining the nearest neighbors used for feature estimation.", default_.getRadiusSearch());
        params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
        FeatureEstimatorType::declare_params(params);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "Cloud to estimate features of.");
        outputs.declare<FeatureCloud> ("output", "Cloud of features.");
        FeatureEstimatorType::declare_io(params, inputs, outputs);
      }

      void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        input_ = inputs["input"];
        output_ = outputs["output"];

        fp.k = params.get<int> ("k_search");
        fp.radius = params.get<double> ("radius_search");
        fp.locator = params.get<int> ("spatial_locator");   

        custom.configure(params, inputs, outputs);
      }

      /* dispatch to handle process */
      struct feature_dispatch : boost::static_visitor<feature_cloud_variant_t>
      {
        FeatureEstimatorType& ft;

        feature_dispatch(FeatureEstimatorType& ft_) : ft(ft_) { }

        template <typename Feature, typename CloudType>
        feature_cloud_variant_t operator()(Feature& f, CloudType& i) const
        {
          return impl(f, i, pcl_takes_point_trait2<Feature, CloudType, typename FeatureEstimatorType::feature::type >());
        }

        template <typename Feature, typename CloudType>
        feature_cloud_variant_t impl(Feature& f, CloudType& i, boost::true_type) const
        {
          ft.process(f);
          typename ::pcl::PointCloud<typename FeatureEstimatorType::feature::type> o;
          f.setInputCloud(i);
          f.compute(o);
          return feature_cloud_variant_t(o.makeShared());
        }

        template <typename Feature, typename CloudType>
        feature_cloud_variant_t impl(Feature& f, CloudType& i, boost::false_type) const
        {
          throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
        }
      };


      int process(const tendrils& inputs, tendrils& outputs)
      {
        xyz_cloud_variant_t cvar = input_->make_variant();
        if(!configured_){
          impl_ = boost::apply_visitor(make_feature_estimator_variant<FeatureEstimatorType::template feature_estimator>(), cvar);
          boost::apply_visitor(feature_configurator(fp, custom), impl_);
          configured_ = true;
        }
        *output_ = boost::apply_visitor(feature_dispatch(custom), impl_, cvar);
        custom.process(inputs, outputs);
        return 0;
      }

      bool configured_;
      ecto::spore<PointCloud> input_;
      ecto::spore<FeatureCloud> output_;
      feature_estimator_variant_t impl_;
    };

  }
}

