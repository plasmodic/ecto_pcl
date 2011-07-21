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
#include <pcl/features/normal_3d.h>

#define DECLARENORMALESTIMATION(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::NormalEstimation< BOOST_PP_TUPLE_ELEM(2, 0, ELEM), pcl::Normal >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARENORMALESTIMATION, ~, ECTO_XYZ_POINT_TYPES) > feature_estimator_variant_t;

#include "features/Feature.hpp"

struct NormalEstimation
{ 
  template <typename Point>
  struct feature_estimator {
    typedef typename ::pcl::NormalEstimation<Point, ::pcl::Normal> type;
  };
  struct feature {
    typedef ::pcl::Normal type;
  };

  static void declare_params(ecto::tendrils& params) { }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs){ }

  NormalEstimation() {}

  template <typename Point>
  void configure(pcl::NormalEstimation<Point, pcl::Normal>& f) {}
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs) { }

  template <typename Point>
  int process(pcl::NormalEstimation<Point,pcl::Normal>& f) { return 0; }
  int process(const tendrils& inputs, tendrils& outputs){ return 0; }

};

ECTO_CELL(ecto_pcl, ecto::pcl::FeatureCell<NormalEstimation>, "NormalEstimation", "Normal estimation");

