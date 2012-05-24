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

#include <boost/python.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

/* enumerations and values to be wrapped */
#include <pcl/sample_consensus/model_types.h>

namespace bp = boost::python;

#define ENUMSAC(r, data, ELEM)                              \
  .value(BOOST_PP_STRINGIZE(BOOST_PP_CAT(SACMODEL_, ELEM)), \
         BOOST_PP_CAT(pcl::SACMODEL_, ELEM))

#define MODELTYPES                              \
  (PLANE)                                       \
  (LINE)                                        \
  (CIRCLE2D)                                    \
  (CIRCLE3D)                                    \
  (SPHERE)                                      \
  (CYLINDER)                                    \
  (CONE)                                        \
  (TORUS)                                       \
  (PARALLEL_LINE)                               \
  (PERPENDICULAR_PLANE)                         \
  (PARALLEL_LINES)                              \
  (NORMAL_PLANE)                                \
  (REGISTRATION)                                \
  (PARALLEL_PLANE)                              \
  (NORMAL_PARALLEL_PLANE)

struct PointCloud2PointCloudT
{
  static void
  declare_params(tendrils& params)
  {
    params.declare<ecto::pcl::Format>("format", "Format of cloud to grab.", ecto::pcl::FORMAT_XYZRGB);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<ecto::pcl::PointCloud>("input", "An variant based PointCloud.");
    outputs.declare<ecto::tendril::none>("output", "An pcl::PointCloud<PointT> type.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    format_ = params["format"];
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int
  process(const tendrils& /*inputs*/, const tendrils& outputs)
  {
    switch (*format_)
    {
      case ecto::pcl::FORMAT_XYZ:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZ> >();
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZRGB> >();
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        output_ << input_->cast<pcl::PointCloud<pcl::PointNormal> >();
        break;
      case ecto::pcl::FORMAT_XYZI:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZI> >();
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        output_ << input_->cast<pcl::PointCloud<pcl::PointXYZRGBA> >();
        break;
      case ecto::pcl::FORMAT_NORMAL:
        output_ << input_->cast<pcl::PointCloud<pcl::PointNormal> >();
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    return ecto::OK;
  }
  ecto::spore<ecto::pcl::Format> format_;
  ecto::spore<ecto::pcl::PointCloud> input_;
  ecto::tendril_ptr output_;
};

struct PointCloudT2PointCloud
{
  static void
  declare_params(tendrils& params)
  {
    params.declare<ecto::pcl::Format>("format", "Format of cloud to grab.", ecto::pcl::FORMAT_XYZRGB);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    std::string doc = "An pcl::PointCloud<PointT> type.";
    switch (params.get<ecto::pcl::Format>("format"))
    {
      case ecto::pcl::FORMAT_XYZ:
        inputs.declare<pcl::PointCloud<pcl::PointXYZ>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        inputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        inputs.declare<pcl::PointCloud<pcl::PointNormal>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZI:
        inputs.declare<pcl::PointCloud<pcl::PointXYZI>::ConstPtr >("input",doc );
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        inputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >("input",doc );
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    outputs.declare<ecto::pcl::PointCloud>("output", "An variant based PointCloud.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    format_ = params["format"];
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int
  process(const tendrils& /*inputs*/, const tendrils& outputs)
  {
    switch (*format_)
    {
      case ecto::pcl::FORMAT_XYZ:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZ>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZRGB:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_POINTNORMAL:
        *output_ = input_->get<pcl::PointCloud<pcl::PointNormal>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZI:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZI>::ConstPtr >();
        break;
      case ecto::pcl::FORMAT_XYZRGBA:
        *output_ = input_->get<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr >();
        break;
      default:
        throw std::runtime_error("Unsupported point cloud type.");
    }
    return ecto::OK;
  }
  ecto::spore<ecto::pcl::Format> format_;
  ecto::spore<ecto::pcl::PointCloud> output_;
  ecto::tendril_ptr input_;
};

ECTO_DEFINE_MODULE(ecto_pcl)
{
  bp::enum_<pcl::SacModel>("SacModel")
    BOOST_PP_SEQ_FOR_EACH(ENUMSAC, ~, MODELTYPES)
    .export_values()
    ;

  bp::enum_<ecto::pcl::Format>("Format")
    .value("XYZ", ecto::pcl::FORMAT_XYZ)
    .value("XYZI", ecto::pcl::FORMAT_XYZI)
    .value("XYZRGB", ecto::pcl::FORMAT_XYZRGB)
    .value("XYZRGBA", ecto::pcl::FORMAT_XYZRGBA)
    .value("XYZRGBNORMAL",ecto::pcl::FORMAT_XYZRGBNORMAL)
    .value("POINTNORMAL",ecto::pcl::FORMAT_POINTNORMAL)

    .value("NORMAL", ecto::pcl::FORMAT_NORMAL)
    .value("PFHSIGNATURE", ecto::pcl::FORMAT_PFHSIGNATURE)
    .value("FPFHSIGNATURE", ecto::pcl::FORMAT_FPFHSIGNATURE)
    .value("VFHSIGNATURE", ecto::pcl::FORMAT_VFHSIGNATURE)
    .export_values()
    ;

  bp::scope().attr("KDTREE_FLANN") = 0;
  bp::scope().attr("KDTREE_ORGANIZED_INDEX") = 1;
}

namespace ecto {
  namespace pcl {
    typedef PointCloud2PointCloudT PointCloud2PointCloudT;
    typedef PointCloudT2PointCloud PointCloudT2PointCloud;
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PointCloud2PointCloudT, "PointCloud2PointCloudT",
          "Convert a generic variant based PointCloud to a strongly typed pcl::PointCloud<pcl::PointT>.")
ECTO_CELL(ecto_pcl, ecto::pcl::PointCloudT2PointCloud, "PointCloudT2PointCloud",
          "Convert a strongly typed pcl::PointCloud<pcl::PointT> to a generic variant based PointCloud.")

