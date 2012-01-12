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
#include <boost/format.hpp>
#include <boost/format/free_funcs.hpp>

namespace ecto {
  namespace pcl {

    struct PCDWriter
    {
      PCDWriter():count_(0){}

      static void declare_params(tendrils& params)
      {
        params.declare<std::string> ("filename_format",
                                     "The format string for saving pcds, "
                                     "must succeed with a single unsigned int argument.",
                                     "cloud_%04u.pcd");
        params.declare<bool> ("binary", "Use binary encoding.", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud>("input", "A point cloud to put in a pcd file.");
        outputs.declare<sensor_msgs::PointCloud2ConstPtr>("cloud_message", "the cloud message used for writing.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input_ = inputs["input"];
        filename_format_ = params["filename_format"];
        binary_ = params["binary"];
        cloud_message_ = outputs["cloud_message"];
      }

      struct write_dispatch : boost::static_visitor<sensor_msgs::PointCloud2ConstPtr>
      {
        std::string file;
        bool binary;
        write_dispatch(std::string f,bool binary = false) : file(f),binary(binary) {}

        template <typename CloudType>
        result_type operator()(CloudType& cloud) const
        {
          if(binary)
            ::pcl::io::savePCDFileBinary(file, *cloud);
          else
          {
            sensor_msgs::PointCloud2Ptr blob(new sensor_msgs::PointCloud2);
            ::pcl::toROSMsg (*cloud, *blob);
            ::pcl::PCDWriter writer;
            writer.writeASCII(file,*blob,cloud->sensor_origin_, cloud->sensor_orientation_,8);
            return blob;
          }
          return sensor_msgs::PointCloud2Ptr();

        }
      };

      int process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        std::string filename = boost::str(boost::format(*filename_format_)%count_++);
        xyz_cloud_variant_t cv = input_->make_variant();
        *cloud_message_ = boost::apply_visitor(write_dispatch(filename,*binary_), cv);
        return 0;
      }

      spore<PointCloud> input_;
      spore<std::string> filename_format_;
      spore<bool> binary_;
      spore<sensor_msgs::PointCloud2ConstPtr> cloud_message_;
      unsigned count_;

    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PCDWriter,
          "PCDWriter", "Write a cloud to a PCD file");

