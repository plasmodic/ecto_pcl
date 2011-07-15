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

namespace ecto {
  namespace pcl {

    template <typename FilterType>
    struct FilterCell
    {
      // typedef typename FilterType::filter_variant_t filter_variant_t;
      FilterType* this_() { return static_cast<FilterType*>(this); }

      struct FilterParams {
        std::string filter_field_name; 
        double filter_limit_min; 
        double filter_limit_max; 
        bool filter_limit_negative; 
      };

      /* used to create a filter */
      template <template <class> class FilterPolicy>
      struct make_filter_variant : boost::static_visitor<filter_variant_t>
      {
        template <typename CloudType >
        filter_variant_t operator()(const CloudType& p) const
        {
          return filter_variant_t(typename FilterPolicy<typename CloudType::element_type::PointType>::type());
        }
      };

      struct filter_configurator : boost::static_visitor<void>
      { 
        FilterParams& fp;
        FilterType& ft;
        filter_configurator(FilterParams& fp_, FilterType& ft_) : fp(fp_), ft(ft_) {}

        template <typename T>
        void operator()(T& impl_) const 
        { 
          impl_.setFilterFieldName(fp.filter_field_name);
          impl_.setFilterLimits(fp.filter_limit_min, fp.filter_limit_max);
          impl_.setFilterLimitsNegative(fp.filter_limit_negative);
          ft.configure(impl_);
        } 
      };

      static void declare_params(ecto::tendrils& params)
      {

        typename FilterType::template filter< ::pcl::PointXYZRGB>::type default_;
        params.declare<std::string> ("filter_field_name", "The name of the field to use for filtering.", "");
        double filter_limit_min, filter_limit_max;
        default_.getFilterLimits(filter_limit_min, filter_limit_max);
        params.declare<double> ("filter_limit_min", "Minimum value for the filter.", filter_limit_min);
        params.declare<double> ("filter_limit_max", "Maximum value for the filter.", filter_limit_max);
        params.declare<bool> ("filter_limit_negative", "To negate the limits or not.", default_.getFilterLimitsNegative());

        FilterType::declare_params(params);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "The cloud to filter");
        outputs.declare<PointCloud> ("output", "Filtered cloud.");
        FilterType::declare_io(params, inputs, outputs);
      }

      void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        input_ = inputs.at("input");
        output_ = outputs.at("output");

        FilterParams fp;

        fp.filter_field_name = params.get<std::string> ("filter_field_name");
        fp.filter_limit_min = params.get<double> ("filter_limit_min");
        fp.filter_limit_max = params.get<double> ("filter_limit_max");
        fp.filter_limit_negative = params.get<bool> ("filter_limit_negative");

        this_()->configure(params, inputs, outputs);

        cloud_variant_t cv = input_->make_variant();
        if(!configured_){
          impl_ = boost::apply_visitor(make_filter_variant<FilterType::template filter>(), cv);
          boost::apply_visitor(filter_configurator(fp, *this_()), impl_);
          configured_ = true;
        }

      }

      /* dispatch to handle process */
      struct filter_dispatch : boost::static_visitor<cloud_variant_t>
      {
        template <typename Filter, typename CloudType>
        cloud_variant_t operator()(Filter& f, CloudType& i) const
        {
          return impl(f, i, filter_takes_point_trait<Filter, CloudType>());
        }

        template <typename Filter, typename CloudType>
        cloud_variant_t impl(Filter& f, boost::shared_ptr<const CloudType>& i, boost::true_type) const
        {
          CloudType o;
          f.setInputCloud(i);
          f.filter(o);
          return cloud_variant_t(o.makeShared());
        }

        template <typename Filter, typename CloudType>
        cloud_variant_t impl(Filter& f, CloudType& i, boost::false_type) const
        {
          throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
        }
      };


      int process(const tendrils& inputs, tendrils& outputs)
      {
        this_()->process(inputs, outputs);
        cloud_variant_t cvar = input_->make_variant();
        *output_ = boost::apply_visitor(filter_dispatch(), impl_, cvar);
        
      }

      bool configured_;
      ecto::spore<PointCloud> input_;
      ecto::spore<PointCloud> output_;
      filter_variant_t impl_;
    };

  }
}
