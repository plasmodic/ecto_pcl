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
#include <ecto_pcl.hpp>

#include <pcl/io/pcd_io.h>

struct PCDWriter
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("filename", "Name of the pcd file", "");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<PointCloud>("input", "A point cloud to put in the bag file.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    input_ = inputs["input"];
    filename_ = params["filename"];
  }

  struct write_dispatch : boost::static_visitor<void>
  {
    std::string file;
    write_dispatch(std::string f) : file(f) {}

    template <typename CloudType>
    void operator()(CloudType& cloud) const
    {
      pcl::io::savePCDFileASCII(file, *cloud);
    }
  };

  int process(const tendrils& /*inputs*/, const tendrils& outputs)
  { 
    xyz_cloud_variant_t cv = input_->make_variant();
    boost::apply_visitor(write_dispatch(*filename_), cv);
    return 0;
  }

  ecto::spore<PointCloud> input_;
  ecto::spore<std::string> filename_;

};

ECTO_CELL(ecto_pcl, PCDWriter, "PCDWriter", "Write a cloud to a PCD file");

