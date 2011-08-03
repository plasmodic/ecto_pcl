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
#include <pcl/surface/convex_hull.h>


#define DECLARECONVEXHULL(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::ConvexHull< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARECONVEXHULL, ~, ECTO_XYZ_POINT_TYPES) > surface_variant_t;

struct ConvexHull
{
  /* used to create a surface */
  template <template <class> class SurfacePolicy>
  struct make_surface_variant : boost::static_visitor<surface_variant_t>
  {
    template <typename CloudType >
    surface_variant_t operator()(const CloudType& p) const
    {
      return surface_variant_t(SurfacePolicy<typename CloudType::element_type::PointType>());
    }
  };

  /* dispatch to handle process */
  struct surface_dispatch : boost::static_visitor<xyz_cloud_variant_t>
  {
    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t operator()(Surface& f, CloudType& i) const
    {
      return impl(f, i, pcl_takes_point_trait<Surface, CloudType>());
    }

    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t impl(Surface& f, boost::shared_ptr<const CloudType>& i, boost::true_type) const
    {
      CloudType o;
      f.setInputCloud(i);
      f.reconstruct(o);
      return xyz_cloud_variant_t(o.makeShared());
    }

    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t impl(Surface& f, CloudType& i, boost::false_type) const
    {
      throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
    }
  };

  static void declare_params(tendrils& params)
  {
    // no params
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<PointCloud> ("input", "The cloud to filter");
    outputs.declare<PointCloud> ("output", "Feature cloud.");
  }

  ConvexHull() : configured_(false) {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set in/out.
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    xyz_cloud_variant_t cvar = input_->make_variant();
    if(!configured_){
      impl_ = boost::apply_visitor(make_surface_variant<pcl::ConvexHull>(), cvar);
      configured_ = true;
    }
    *output_ = boost::apply_visitor(surface_dispatch(), impl_, cvar);
    return 0;
  }
  
  bool configured_;
  surface_variant_t impl_;
  ecto::spore<PointCloud> input_;
  ecto::spore<PointCloud> output_;

};

ECTO_CELL(ecto_pcl, ConvexHull, "ConvexHull", "Using libqhull library.");

