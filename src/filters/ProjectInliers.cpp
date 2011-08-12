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
#include <pcl/filters/project_inliers.h>

#define DECLAREPROJECTINLIERS(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::ProjectInliers< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLAREPROJECTINLIERS, ~, ECTO_XYZ_POINT_TYPES) > filter_variant_t;

#include "filters/Filter.hpp"

struct ProjectInliers
{
  template <typename Point>
  struct filter {
    typedef typename ::pcl::ProjectInliers<Point> type;
  };

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int> ("model_type", "The type of model to use.", 0);
    params.declare<bool> ("copy_all_data", "Sets whether all data will be returned, or only the projected inliers.", false);
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare<model_t::ConstPtr> ("model", "Model to use for projection.");
  }

  ProjectInliers() {}

  template <typename Point>
  void configure(pcl::ProjectInliers<Point>& f)
  {
    f.setModelType(model_type);
    f.setCopyAllData(copy_all);
  }
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    model_type = params.get<int> ("model_type");
    copy_all = params.get<bool> ("copy_all_data");
    model_ = inputs["model"];
  }

  template <typename Point>
  int process(pcl::ProjectInliers<Point>& f)
  {
    f.setModelCoefficients(*model_);
    return 0;
  }
  int process(const tendrils& inputs, const tendrils& outputs){ return 0; }

  ecto::spore<model_t::ConstPtr> model_;
  int model_type;
  bool copy_all;

};

ECTO_CELL(ecto_pcl, ecto::pcl::FilterCell<ProjectInliers>, "ProjectInliers", "...");


