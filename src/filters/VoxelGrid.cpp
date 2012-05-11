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
#include <pcl/filters/voxel_grid.h>

namespace ecto {
  namespace pcl {

    struct VoxelGrid
    {
      static void declare_params(tendrils& params)
      {
        ::pcl::VoxelGrid< ::pcl::PointXYZ > default_;
        params.declare<std::string> ("filter_field_name", "The name of the field to use for filtering.", "");
        double filter_limit_min, filter_limit_max;
        default_.getFilterLimits(filter_limit_min, filter_limit_max);
        params.declare<double> ("filter_limit_min", "Minimum value for the filter.", filter_limit_min);
        params.declare<double> ("filter_limit_max", "Maximum value for the filter.", filter_limit_max);
        params.declare<bool> ("filter_limit_negative", "To negate the limits or not.", default_.getFilterLimitsNegative());
        params.declare<float> ("leaf_size", "The size of the leaf(meters), smaller means more points...", 0.05);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud> ("output", "Filtered Cloud.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        filter_field_name_ = params["filter_field_name"];
        filter_limit_min_ = params["filter_limit_min"];
        filter_limit_max_ = params["filter_limit_max"];
        filter_limit_negative_ = params["filter_limit_negative"];
        leaf_size = params["leaf_size"];

        output_ = outputs["output"];
      }
      
      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs, 
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::VoxelGrid<Point> filter;
        filter.setFilterFieldName(*filter_field_name_);
        filter.setFilterLimits(*filter_limit_min_, *filter_limit_max_);
        filter.setFilterLimitsNegative(*filter_limit_negative_);
        filter.setLeafSize(*leaf_size, *leaf_size, *leaf_size);
        filter.setInputCloud(input);

        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        filter.filter(*cloud);
        cloud->header = input->header;
        *output_ = xyz_cloud_variant_t(cloud);

        return ecto::OK;
      }

      ecto::spore<std::string> filter_field_name_;
      ecto::spore<double> filter_limit_min_;
      ecto::spore<double> filter_limit_max_;
      ecto::spore<bool> filter_limit_negative_;
      ecto::spore<float> leaf_size;
      ecto::spore<ecto::pcl::PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::VoxelGrid>,
          "VoxelGrid", "Voxel grid filter");

