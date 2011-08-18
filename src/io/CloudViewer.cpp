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
#include <pcl/visualization/cloud_viewer.h>

#include <boost/variant/get.hpp>

namespace ecto {
  namespace pcl {

    struct CloudViewer
    {
      static void declare_params(tendrils& params)
      {
        params.declare<std::string> ("window_name", "The window name", "cloud viewer");

      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "The cloud to view");
        outputs.declare<bool> ("stop", "True if stop requested", false);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        viewer_.reset(new ::pcl::visualization::CloudViewer(params.get<std::string> ("window_name")));
      }

      int process(const tendrils& inputs, const tendrils& outputs)
      {
        if (!viewer_)
          return 1;

        PointCloud cloud = inputs.get<PointCloud> ("input");
        xyz_cloud_variant_t cv = cloud.make_variant();
        try{
          CloudPOINTXYZRGB::ConstPtr c = boost::get<CloudPOINTXYZRGB::ConstPtr>(cv);
          if(c)
            viewer_->showCloud(c, "cloud");
        }catch(boost::bad_get){
          try{
            CloudPOINTXYZ::ConstPtr c = boost::get<CloudPOINTXYZ::ConstPtr>(cv);
            if(c)
              viewer_->showCloud(c, "cloud");
          }catch(boost::bad_get){
            throw std::runtime_error("CloudViewer supports only XYZ and XYZRGB point clouds!");
          }
        }
        if (viewer_->wasStopped(10))
          outputs.get<bool> ("stop") = true;
        return 0;
      }
      boost::shared_ptr< ::pcl::visualization::CloudViewer > viewer_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::CloudViewer, "CloudViewer", "Viewer of clouds");

