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
#include <fstream>
#include <boost/format.hpp>
#include <boost/format/free_funcs.hpp>
namespace ecto
{
  namespace pcl
  {

    template<typename PointT>
    inline void
    writePLY(const ::pcl::PointCloud<PointT>& cloud_m, const std::string& mesh_file_name)
    {
      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                "element vertex "
                << cloud_m.points.size() << "\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "end_header\n";

      //<x> <y> <z> <r> <g> <b>
      for (size_t i = 0; i < cloud_m.points.size(); i++)
      {
        const PointT& p = cloud_m.points[i];
        mesh_file << p.x << " " << p.y << " " << p.z << "\n";
      }
    }
    template<>
    inline void
    writePLY<CloudPOINTXYZRGB::PointType>(const CloudPOINTXYZRGB& cloud_m, const std::string& mesh_file_name)
    {
      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                "element vertex "
                << cloud_m.points.size() << "\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property uchar red\n"
                "property uchar green\n"
                "property uchar blue\n"
                "end_header\n";
      //<x> <y> <z> <r> <g> <b>
      for (size_t i = 0; i < cloud_m.points.size(); i++)
      {
        const CloudPOINTXYZRGB::PointType& p = cloud_m.points[i];
        mesh_file << p.x << " " << p.y << " " << p.z << " " << int(p.r) << " " << int(p.g) << " " << int(p.b) << "\n";
      }
    }

    template<>
       inline void
       writePLY<CloudPOINTNORMAL::PointType>(const CloudPOINTNORMAL& cloud_m,
                                                   const std::string& mesh_file_name)
       {
         std::ofstream mesh_file(std::string(mesh_file_name).c_str());
         mesh_file << "ply\n"
                   "format ascii 1.0\n"
                   "element vertex "
                   << cloud_m.points.size() << "\n"
                   "property float x\n"
                   "property float y\n"
                   "property float z\n"
                   "property float nx\n"
                   "property float ny\n"
                   "property float nz\n"
                   "end_header\n";

         //<x> <y> <z> <r> <g> <b>
         for (size_t i = 0; i < cloud_m.points.size(); i++)
         {
           const CloudPOINTNORMAL::PointType& p = cloud_m.points[i];
           mesh_file << p.x << " " << p.y << " " << p.z << " "
                     << p.normal_x << " " << p.normal_y << " " << p.normal_z << "\n";
         }
       }

    template<>
    inline void
    writePLY<CloudPOINTXYZRGBNORMAL::PointType>(const CloudPOINTXYZRGBNORMAL& cloud_m,
                                                const std::string& mesh_file_name)
    {
      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                "element vertex "
                << cloud_m.points.size() << "\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property uchar red\n"
                "property uchar green\n"
                "property uchar blue\n"
                "property float nx\n"
                "property float ny\n"
                "property float nz\n"
                "end_header\n";

      //<x> <y> <z> <r> <g> <b>
      for (size_t i = 0; i < cloud_m.points.size(); i++)
      {
        const CloudPOINTXYZRGBNORMAL::PointType& p = cloud_m.points[i];
        mesh_file << p.x << " " << p.y << " " << p.z << " "
                  << int(p.r) << " " << int(p.g) << " " << int(p.b) << " "
                  << p.normal_x << " " << p.normal_y << " " << p.normal_z << "\n";
      }
    }

    struct PLYWriter
    {
      PLYWriter()
          :
            count_(0)
      {
      }

      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("filename_format", "The format string for saving PLY files, "
                                    "must succeed with a single unsigned int argument.",
                                    "cloud_%04u.ply");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud>("input", "A point cloud to put in a pcd file.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input_ = inputs["input"];
        filename_format_ = params["filename_format"];
      }

      struct write_dispatch: boost::static_visitor<>
      {
        std::string file;
        write_dispatch(std::string f)
            :
              file(f)
        {
        }
        template<typename CloudType>
        void
        operator()(CloudType& cloud) const
        {
          writePLY(*cloud, file);
        }
      };

      int
      process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
      {
        std::string filename = boost::str(boost::format(*filename_format_) % count_++);
        xyz_cloud_variant_t cv = input_->make_variant();
        boost::apply_visitor(write_dispatch(filename), cv);
        return ecto::OK;
      }
      spore<PointCloud> input_;
      spore<std::string> filename_format_;
      unsigned count_;
    };
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PLYWriter, "PLYWriter", "Write a cloud to a PLY file");

