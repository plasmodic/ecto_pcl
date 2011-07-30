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
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#define DECLAREEXTRACT(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) pcl::ExtractPolygonalPrismData< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLAREEXTRACT, ~, ECTO_XYZ_POINT_TYPES) > segmentation_variant_t;

#include <segmentation/Segmentation.hpp>
#include <boost/variant/get.hpp>

struct ExtractPolygonalPrismData
{
  template <typename Point>
  struct segmentation {
    typedef typename ::pcl::ExtractPolygonalPrismData<Point> type;
  };

  static void declare_params(ecto::tendrils& params)
  {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> default_;
    double h_min, h_max;
    default_.getHeightLimits (h_min, h_max);
    params.declare<double> ("height_min", "Minimum allowable height limits for the model.", h_min);
    params.declare<double> ("height_max", "Maximum allowable height limits for the model.", h_max);
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare<PointCloud> ("planar_hull", "Planar hull to use.");
    outputs.declare<indices_t::ConstPtr> ("inliers", "Inliers of the model.");
  }

  template <typename Point>
  void configure(pcl::ExtractPolygonalPrismData<Point>& impl_)
  {
    impl_.setHeightLimits (height_min, height_max);
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    hull_ = inputs["planar_hull"];
    inliers_ = outputs["inliers"];

    height_min = params.get<double> ("height_min");
    height_max = params.get<double> ("height_max");
  }

  template <typename Point>
  int process(pcl::ExtractPolygonalPrismData<Point>& impl_) { 
    indices_t::Ptr inliers ( new indices_t() );

    // copy header
    //inliers->header = (*input_)->header;

    try{
      xyz_cloud_variant_t cv = hull_->make_variant();
      typename pcl::PointCloud<Point>::ConstPtr hull = boost::get<typename pcl::PointCloud<Point>::ConstPtr>(cv);
      if(hull)
        impl_.setInputPlanarHull (hull);
    }catch(boost::bad_get){
      throw std::runtime_error("Failure to set hull input.");
    }
    impl_.segment (*inliers);

    *inliers_ = inliers;
    return 0;
  }
  int process(const tendrils& inputs, tendrils& outputs) { return 0; }

  double height_min, height_max;
  ecto::spore< PointCloud > hull_;
  ecto::spore< indices_t::ConstPtr > inliers_;

};

ECTO_CELL(ecto_pcl, ecto::pcl::SegmentationCell<ExtractPolygonalPrismData>, "ExtractPolygonalPrismData", "Uses a set of point indices that respresent a planar model, and together with a given height, generates a 3D polygonal prism.");

