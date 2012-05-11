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
#include <pcl/filters/extract_indices.h>

namespace ecto {
  namespace pcl {

    struct ExtractIndices
    {
      static void declare_params(tendrils& params)
      {
        params.declare<bool> ("negative", "Sets whether the indices should be returned, or all points _except_ the indices.", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<Indices::ConstPtr> ("indices", "The indices to extract.");
        outputs.declare<PointCloud> ("output", "Filtered Cloud.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        negative_ = params["negative"];
        indices_ = inputs["indices"];
        output_ = outputs["output"];
      }
      
      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs, 
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::ExtractIndices<Point> filter;
        filter.setNegative(*negative_);
        filter.setIndices(*indices_);
        filter.setInputCloud(input);
              
        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        filter.filter(*cloud);
        cloud->header = input->header;
        *output_ = xyz_cloud_variant_t(cloud);

        return OK;
      }

      spore<bool> negative_;
      spore<Indices::ConstPtr> indices_;
      spore<PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::ExtractIndices>,
          "ExtractIndices", "Extract a new cloud given an existing cloud and a set of indices to extract.");

