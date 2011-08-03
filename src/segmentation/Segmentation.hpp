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

    template <typename SegmentationType>
    struct SegmentationCell
    {
      SegmentationType custom;

      template <template <class> class SegmentationPolicy>
      struct make_segmentation_variant : boost::static_visitor<segmentation_variant_t>
      {
        template <typename CloudType >
        segmentation_variant_t operator()(const CloudType& p) const
        {
          return segmentation_variant_t(typename SegmentationPolicy<typename CloudType::element_type::PointType>::type() );
        }
      };

      struct segmentation_configurator : boost::static_visitor<void>
      { 
        SegmentationType& st;
        segmentation_configurator(SegmentationType st_) : st(st_) {}

        template <typename T>
        void operator()(T& impl_) const 
        { 
          st.configure(impl_);
        }
      };

      static void declare_params(ecto::tendrils& params)
      {
        SegmentationType::declare_params(params);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "The cloud to segment.");
        SegmentationType::declare_io(params, inputs, outputs);
      }

      SegmentationCell() : configured_(false) {}

      void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        input_ = inputs["input"];
        custom.configure(params, inputs, outputs);
      }

      /* dispatch to handle process */
      struct segmentation_dispatch : boost::static_visitor< void >
      {
        SegmentationType& st;

        segmentation_dispatch(SegmentationType& st_) : st(st_) { }

        template <typename Segmentation, typename CloudType>
        void operator()(Segmentation& s, CloudType& i) const
        {
          return impl(s, i, pcl_takes_point_trait<Segmentation, CloudType>());
        }

        template <typename Segmentation, typename CloudType>
        void impl(Segmentation& s, boost::shared_ptr<const CloudType>& i, boost::true_type) const
        {
          s.setInputCloud(i);
          st.process(s);
        }

        template <typename Segmentation, typename CloudType>
        void impl(Segmentation& s, CloudType& i, boost::false_type) const
        {
          return impl2(s, i, pcl_takes_point_trait2<Segmentation, CloudType,::pcl::Normal >());
        }

        template <typename Segmentation, typename CloudType>
        void impl2(Segmentation& s, CloudType& i, boost::true_type) const
        {
          s.setInputCloud(i);
          st.process(s);
        }

        template <typename Feature, typename CloudType>
        void impl2(Feature& f, CloudType& i, boost::false_type) const
        {
          throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
        }
      };

      int process(const tendrils& inputs, tendrils& outputs)
      {
        xyz_cloud_variant_t cvar = input_->make_variant();
        if(!configured_){
          impl_ = boost::apply_visitor(make_segmentation_variant<SegmentationType::template segmentation>(), cvar);
          boost::apply_visitor(segmentation_configurator(custom), impl_);
          configured_ = true;
        }
        boost::apply_visitor(segmentation_dispatch(custom), impl_, cvar);
        return 0;
      }

      bool configured_;
      ecto::spore<PointCloud> input_;
      segmentation_variant_t impl_;

    };

  }
}
