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

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <pcl/io/pcd_io.h>

namespace ecto {
  namespace pcl {

    struct PCDReader
    {
      static void declare_params(tendrils& params)
      {
        params.declare<ecto::pcl::Format>("format", "Format of cloud found in PCD file.",
                                           ecto::pcl::FORMAT_XYZRGB);
        params.declare<std::string> ("filename", "Name of the pcd file", "");
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud>("output", "A point cloud from the pcd file.");
      }

      PCDReader() { first = true; }
      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        output_ = outputs["output"];
        format_ = params["format"];
        filename_ = params["filename"];
      }

      int process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        if (!first)
          return OK;
        first = false;
        switch(*format_)
        {
          case FORMAT_XYZ:
            {
              std::cout << "opening " << *filename_ << std::endl;
              ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZ >);
              if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZ >(*filename_, *cloud) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZ cloud.");
                return 1;
              }
              std::cout << "Made it this far" << std::endl;
              PointCloud p( cloud );
              *output_ = p;
            } break;
          case FORMAT_XYZRGB:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
              if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZRGB > (*filename_, *cloud) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZRGB cloud.");
                return 1;
              }
              PointCloud p( cloud );
              *output_ = p;
            } break;
          default:
            throw std::runtime_error("PCDReader: Unknown cloud type.");
        }
        return OK;
      }

      bool first;
      spore<PointCloud> output_;
      spore<ecto::pcl::Format> format_;
      spore<std::string> filename_;

    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PCDReader, "PCDReader", "Read a cloud from a PCD file");

