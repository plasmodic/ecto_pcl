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
#include <pcl/filters/voxel_grid.h>

#define DECLAREVOXELGRID(r, data, i, ELEM)                              \
  BOOST_PP_COMMA_IF(i) BOOST_PP_CAT(pcl::VoxelGrid<pcl::Point,ELEM)>

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLAREVOXELGRID, ~, POINTTYPES) > filter_variant_t;

#include "FilterCell.hpp"

struct VoxelGrid
{
  template <typename Point>
  struct filter {
    typedef typename ::pcl::VoxelGrid<Point> type;
  };

  /*
  struct filter_configurator : boost::static_visitor<void>
  { 
    filter_params& fp;
    filter_configurator(filter_params& fp_) : fp(fp_) {}

    template <typename PointType>
    void operator()(pcl::VoxelGrid<PointType>& impl_) const
    {
      impl_.setLeafSize(fp.leaf_size, fp.leaf_size, fp.leaf_size);
    }
  };

  */
  static void declare_params(ecto::tendrils& params)
  {
    // custom params
    params.declare<float> ("leaf_size", "The size of the leaf(meters), smaller means more points...", 0.05);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) { }

  VoxelGrid() {}

  template <typename Point>
  void configure(pcl::VoxelGrid<Point>& f)
  {
    f.setLeafSize(leaf_size, leaf_size, leaf_size);
  }

  template <typename Point>
  void process(pcl::VoxelGrid<Point>& f)
  {
    // can set indices here
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set in/out.
    // instantiate and set filter details.
    leaf_size = params.get<float> ("leaf_size");
    
    //    cloud_variant_t cv = input_->make_variant();
    //    if(!configured_){
    //      impl_ = boost::apply_visitor(make_filter_variant<pcl::VoxelGrid>(), cv);
    //      boost::apply_visitor(filter_configurator(fp), impl_);
    //      configured_ = true;
    //    }
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    return 0;
  }

  double leaf_size;

};

ECTO_CELL(ecto_pcl, ecto::pcl::FilterCell<VoxelGrid>, "VoxelGrid", "Voxel grid filter");


