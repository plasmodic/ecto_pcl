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
#include <pcl/segmentation/extract_clusters.h>

#define DECLARECLUSTERS(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::EuclideanClusterExtraction< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARECLUSTERS, ~, ECTO_XYZ_POINT_TYPES) > segmentation_variant_t;

#include <segmentation/Segmentation.hpp>
#include <boost/variant/get.hpp>

struct EuclideanClusterExtraction
{
  template <typename Point>
  struct segmentation {
    typedef typename ::pcl::EuclideanClusterExtraction<Point> type;
  };

  static void declare_params(ecto::tendrils& params)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> default_;
    params.declare<double> ("cluster_tolerance", "Spatial cluster tolerance as a measure in the L2 Euclidean space.", 0.05);
    params.declare<int> ("min_cluster_size", "Minimum number of points that a cluster needs to contain in order to be considered valid.", 1);
    params.declare<int> ("max_cluster_size", "Maximum number of points that a cluster needs to contain in order to be considered valid.", default_.getMaxClusterSize());
    params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    outputs.declare<cluster_t> ("output", "Clusters.");
  }

  template <typename Point>
  void configure(pcl::EuclideanClusterExtraction<Point>& impl_)
  {
    impl_.setClusterTolerance (cluster_tolerance);
    impl_.setMinClusterSize (min_cluster_size);
    impl_.setMaxClusterSize (max_cluster_size);
    //impl_.setSpatialLocator (locator);
  }
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    output_ = outputs["output"];

    cluster_tolerance = params.get<double> ("cluster_tolerance"); 
    min_cluster_size = params.get<int> ("min_cluster_size"); 
    max_cluster_size = params.get<int> ("max_cluster_size"); 
    //locator = params.get<int> ("spatial_locator");    
  }

  template <typename Point>
  int process(pcl::EuclideanClusterExtraction<Point>& impl_) {
    cluster_t clusters;
    impl_.extract (clusters);
    *output_ = clusters;
    return 0;
  }
  int process(const tendrils& inputs, const tendrils& outputs) { return 0; }

  double cluster_tolerance;
  int min_cluster_size; 
  int max_cluster_size; 
  //int locator;    
  
  ecto::spore< cluster_t > output_;

};

ECTO_CELL(ecto_pcl, ecto::pcl::SegmentationCell<EuclideanClusterExtraction>, "EuclideanClusterExtraction", "Segmentation for cluster extraction in a Euclidean sense.");

