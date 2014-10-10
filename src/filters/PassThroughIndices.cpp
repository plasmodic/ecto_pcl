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
 *
 * Adjusted by Michael Goerner in 2014 to filter indices instead of point clouds
 */

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <pcl/filters/passthrough.h>

namespace ecto {
  namespace pcl {

    struct PassThroughIndices
    {
      static void declare_params(tendrils& params)
      {
        ::pcl::PassThrough< ::pcl::PointXYZ > default_;
        params.declare(&PassThroughIndices::filter_field_name_, "filter_field_name", "The name of the field to use for filtering.", "");
        float filter_limit_min, filter_limit_max;
        default_.getFilterLimits(filter_limit_min, filter_limit_max);
        params.declare(&PassThroughIndices::filter_limit_min_, "filter_limit_min", "Minimum value for the filter.", filter_limit_min);
        params.declare(&PassThroughIndices::filter_limit_max_, "filter_limit_max", "Maximum value for the filter.", filter_limit_max);
        params.declare(&PassThroughIndices::filter_limit_negative_, "filter_limit_negative", "To negate the filter limits.", default_.getFilterLimitsNegative());
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&PassThroughIndices::indices_, "indices", "relevant indices of the input cloud [optional].").required(false);
        outputs.declare(&PassThroughIndices::output_, "output", "filtered point indices");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {}

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::PointIndices::Ptr idx(new ::pcl::PointIndices);

        ::pcl::PassThrough<Point> filter;
        filter.setFilterFieldName(*filter_field_name_);
        filter.setFilterLimits(*filter_limit_min_, *filter_limit_max_);
        filter.setFilterLimitsNegative(*filter_limit_negative_);
        filter.setInputCloud(input);
        if(indices_.user_supplied())
          filter.setIndices(*indices_);

        filter.filter(idx->indices);

        idx->header= input->header;
        *output_= idx;

        return ecto::OK;
      }

      ecto::spore<std::string> filter_field_name_;
      ecto::spore<double> filter_limit_min_;
      ecto::spore<double> filter_limit_max_;
      ecto::spore<bool> filter_limit_negative_;

      ecto::spore< ::pcl::PointIndices::ConstPtr > indices_;

      ecto::spore< ::pcl::PointIndices::ConstPtr > output_;
    };
  }
}
ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::PassThroughIndices>,
          "PassThroughIndices", "PassThrough filter that returns PointIndices");
