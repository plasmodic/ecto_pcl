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
#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace ecto {
  namespace pcl {
    struct NormalEstimation
    {
      static void declare_params(ecto::tendrils& params)
      {
        params.declare<int> ("k_search", "The number of k nearest neighbors to use for feature estimation.", 0);
        params.declare<double> ("radius_search", "The sphere radius to use for determining the nearest neighbors used for feature estimation.", 0);
        params.declare<int> ("spatial_locator", "The search method to use: FLANN(0), ORGANIZED(1).",0);
        params.declare<double> ("vp_x", "Viewpoint x component.",0);
        params.declare<double> ("vp_y", "Viewpoint y component.",0);
        params.declare<double> ("vp_z", "Viewpoint z component.",0);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::FeatureCloud> ("output", "Cloud of features.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        k_ = params["k_search"];
        radius_ = params["radius_search"];
        locator_ = params["spatial_locator"];
        output_ = outputs["output"];
        vp_x_ = params["vp_x"];
        vp_y_ = params["vp_y"];
        vp_z_ = params["vp_z"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::NormalEstimation<Point, ::pcl::Normal> impl;
        typename ::pcl::PointCloud< ::pcl::Normal >::Ptr normals(new typename ::pcl::PointCloud< ::pcl::Normal >);

        impl.setKSearch(*k_);
        impl.setRadiusSearch(*radius_);
        typename ::pcl::search::Search<Point>::Ptr tree_;
        switch (*locator_)
        {
          case 0:
          {
            tree_.reset (new ::pcl::search::KdTree<Point>);
            break;
          }
          case 1:
          {
            tree_.reset (new ::pcl::search::OrganizedNeighbor<Point>);
            break;
          }
        }
        impl.setSearchMethod(tree_);
        impl.setInputCloud(input);
        impl.setViewPoint(*vp_x_,*vp_y_,*vp_z_);
        impl.compute(*normals);
        normals->header = input->header;
        *output_ = ecto::pcl::feature_cloud_variant_t(normals);
        return ecto::OK;
      }

      ecto::spore<int> k_;
      ecto::spore<double> radius_,vp_x_,vp_y_,vp_z_;
      ecto::spore<int> locator_;
      ecto::spore<ecto::pcl::FeatureCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::NormalEstimation>, "NormalEstimation", "Normal estimation");

