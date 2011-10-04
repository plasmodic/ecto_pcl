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

#pragma once

#include <ecto/ecto.hpp>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/preprocessor/cat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#define ECTO_XYZ_POINT_TYPES                         \
  ((pcl::PointXYZRGB, POINTXYZRGB))                  \
  ((pcl::PointXYZ, POINTXYZ))                        \
  ((pcl::PointNormal, POINTNORMAL))                  \
  ((pcl::PointXYZI, POINTXYZI))                      \
  ((pcl::PointXYZRGBA, POINTXYZRGBA))                \
  ((pcl::PointXYZRGBNormal, POINTXYZRGBNORMAL))

#define ECTO_FEATURE_POINT_TYPES                     \
  ((pcl::Normal, NORMAL))                            \
  ((pcl::PFHSignature125, PFHSIGNATURE125))          \
  ((pcl::FPFHSignature33, FPFHSIGNATURE33))          \
  ((pcl::VFHSignature308, VFHSIGNATURE308))          \
  ((pcl::Narf36, NARF36))

#define DECLARECLOUD(r, data, ELEM) \
  typedef pcl::PointCloud< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) > BOOST_PP_CAT(Cloud, BOOST_PP_TUPLE_ELEM(2, 1, ELEM));
BOOST_PP_SEQ_FOR_EACH(DECLARECLOUD, ~, ECTO_XYZ_POINT_TYPES);
BOOST_PP_SEQ_FOR_EACH(DECLARECLOUD, ~, ECTO_FEATURE_POINT_TYPES);

#define DECLARECLOUDVARIANT(r, data, i, ELEM) \
  BOOST_PP_COMMA_IF(i) BOOST_PP_CAT(Cloud, BOOST_PP_TUPLE_ELEM(2, 1, ELEM))::ConstPtr

namespace ecto{
  namespace pcl {

    typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARECLOUDVARIANT, ~, ECTO_XYZ_POINT_TYPES) > xyz_cloud_variant_t;
    typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARECLOUDVARIANT, ~, ECTO_FEATURE_POINT_TYPES) > feature_cloud_variant_t;

    struct PointCloud {
      struct holder_base { virtual xyz_cloud_variant_t make_variant() = 0; };

      template <typename T>
      struct holder : holder_base
      {
        T t;
        holder(T t_) : t(t_) { }

        xyz_cloud_variant_t make_variant() { return xyz_cloud_variant_t(t); }
      };

      boost::shared_ptr<holder_base> held;

      template <typename T>
      PointCloud(T t_)
      {
        held.reset(new holder<T>(t_));
      }

      PointCloud() {}
      xyz_cloud_variant_t make_variant() { return held->make_variant(); }

      template<typename T>
      typename T::ConstPtr cast() { return boost::get<typename T::ConstPtr>(held->make_variant());}
    };

    struct FeatureCloud {
      struct holder_base { virtual feature_cloud_variant_t make_variant() = 0; };

      template <typename T>
      struct holder : holder_base
      {
        T t;
        holder(T t_) : t(t_) { }

        feature_cloud_variant_t make_variant() { return feature_cloud_variant_t(t); }
      };

      boost::shared_ptr<holder_base> held;

      template <typename T>
      FeatureCloud(T t_)
      {
        held.reset(new holder<T>(t_));
      }
      FeatureCloud() {}
      feature_cloud_variant_t make_variant() { return held->make_variant();  }
    };

    typedef ::pcl::PointIndices Indices;
    typedef ::pcl::ModelCoefficients ModelCoefficients;
    typedef std::vector< ::pcl::PointIndices > Clusters;

    enum Format
    {
      FORMAT_XYZ,
      FORMAT_XYZI,
      FORMAT_XYZRGB,
      FORMAT_XYZRGBA,
      FORMAT_XYZRGBNORMAL,
      FORMAT_POINTNORMAL,
      FORMAT_NORMAL,
      FORMAT_PFHSIGNATURE,
      FORMAT_FPFHSIGNATURE,
      FORMAT_VFHSIGNATURE
    };

  }
}

// hacky pcl workaround for private PointCloud
template <typename PclType, typename PointType>
struct pcl_takes_point_trait : boost::false_type {};
template <template <class> class PclType, typename PointType>
struct pcl_takes_point_trait<PclType<PointType>, boost::shared_ptr<const pcl::PointCloud<PointType> > > : boost::true_type {};

template <typename PclType, typename PointIn, typename PointOut>
struct pcl_takes_point_trait2 : boost::false_type {};
template <template <class, class> class PclType, typename PointIn, typename PointOut>
struct pcl_takes_point_trait2<PclType<PointIn, PointOut>, boost::shared_ptr<const pcl::PointCloud<PointIn> >, PointOut > : boost::true_type {};

using ecto::tendrils;

