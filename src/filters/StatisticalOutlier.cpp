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
#include <pcl/filters/statistical_outlier_removal.h>

namespace ecto {
  namespace pcl {

    struct StatisticalOutlierRemoval
    {
      static void declare_params(tendrils& params)
      {
        ::pcl::StatisticalOutlierRemoval< ::pcl::PointXYZ > default_;
        params.declare<int> ("mean_k", "The number of points to use for mean distance estimation.", default_.getMeanK());
        params.declare<double> ("stddev", "The standard deviation multiplier threshold.", default_.getStddevMulThresh());
        params.declare<bool> ("negative", "Sets whether the indices should be returned, or all points _except_ the indices.", default_.getNegative());
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud> ("output", "Filtered Cloud.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        negative_ = params["negative"];
        mean_k_ = params["mean_k"];
        stddev_ = params["stddev"];

        output_ = outputs["output"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        *output_ = xyz_cloud_variant_t(cloud);

        ::pcl::StatisticalOutlierRemoval<Point> filter;
        if(input->size() < 1)
          return ecto::OK;
        filter.setInputCloud(input);
        filter.setMeanK(*mean_k_);
        filter.setStddevMulThresh(*stddev_);
        filter.setNegative(*negative_);

        filter.filter(*cloud);
        cloud->header = input->header;
        *output_ = xyz_cloud_variant_t(cloud);

        return OK;
      }

      spore<int> mean_k_;
      spore<double> stddev_;
      spore<bool> negative_;
      spore<PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::StatisticalOutlierRemoval>,
          "StatisticalOutlierRemoval", "Remove noisy measurements.");

