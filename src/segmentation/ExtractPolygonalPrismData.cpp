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
#include <ecto_pcl/pcl_cell_dual_inputs.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

namespace ecto {
  namespace pcl {

    struct ExtractPolygonalPrismData
    {
      static const std::string SecondInputName;
      static const std::string SecondInputDescription;

      static void declare_params(tendrils& params)
      {
        params.declare<double> ("height_min", "Minimum allowable height limits for the model.", 0.0);
        params.declare<double> ("height_max", "Maximum allowable height limits for the model.", 1.0);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<Indices::ConstPtr> ("inliers", "Inliers of the model.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        height_min_ = params["height_min"];
        height_max_ = params["height_max"];
        inliers_ = outputs["inliers"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs, 
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input1,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input2)
      {
        ::pcl::ExtractPolygonalPrismData<Point> impl;    
        Indices::Ptr inliers ( new Indices() );

        impl.setHeightLimits (*height_min_, *height_max_);
        impl.setInputPlanarHull(input2);
        impl.setInputCloud(input1);
        impl.segment(*inliers);

        *inliers_ = inliers;
        return OK;
      }

      spore<double> height_min_;
      spore<double> height_max_;
      spore<Indices::ConstPtr> inliers_;
    };

    const std::string ExtractPolygonalPrismData::SecondInputName = "planar_hull";
    const std::string ExtractPolygonalPrismData::SecondInputDescription = "Planar hull to use.";

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCellDualInputs<ecto::pcl::ExtractPolygonalPrismData>,
          "ExtractPolygonalPrismData", "Uses a set of point indices that respresent a \
           planar model, and together with a given height, generates a 3D polygonal prism.");

