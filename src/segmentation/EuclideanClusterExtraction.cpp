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
#include <pcl/segmentation/extract_clusters.h>

namespace ecto {
  namespace pcl {

    struct EuclideanClusterExtraction
    {
      static void declare_params(tendrils& params)
      {
        ::pcl::EuclideanClusterExtraction< ::pcl::PointXYZ > default_;
        params.declare<double> ("cluster_tolerance",
                                "Spatial cluster tolerance as a measure in the L2 Euclidean space.", 0.05);
        params.declare<int> ("min_cluster_size",
                             "Minimum number of points that a cluster needs to contain"
                             "in order to be considered valid.", 1);
        params.declare<int> ("max_cluster_size",
                             "Maximum number of points that a cluster needs to contain"
                             "in order to be considered valid.", default_.getMaxClusterSize());
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        outputs.declare<Clusters> ("output", "Clusters.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        cluster_tolerance_ = params["cluster_tolerance"];
        min_cluster_size_ = params["min_cluster_size"];
        max_cluster_size_ = params["max_cluster_size"];

        output_ = outputs["output"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::EuclideanClusterExtraction<Point> impl;
        output_->resize(0);

        impl.setClusterTolerance (*cluster_tolerance_);
        impl.setMinClusterSize (*min_cluster_size_);
        impl.setMaxClusterSize (*max_cluster_size_);
        impl.setInputCloud(input);
        impl.extract (*output_);

        return OK;
      }

      spore<double> cluster_tolerance_;
      spore<int> min_cluster_size_;
      spore<int> max_cluster_size_;
      spore<Clusters> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::EuclideanClusterExtraction>,
          "EuclideanClusterExtraction", "Segmentation for cluster extraction in a Euclidean sense.");

