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

#include "ecto_pcl_prelude.hpp"

#define DECLARECLOUD(r, data, ELEM)                                     \
  typedef pcl::PointCloud<BOOST_PP_CAT(pcl::Point, ELEM)> BOOST_PP_CAT(PointCloud, ELEM);
BOOST_PP_SEQ_FOR_EACH(DECLARECLOUD, ~, POINTTYPES);

#define DECLARECLOUDVARIANT(r, data, i, ELEM)                   \
  BOOST_PP_COMMA_IF(i) BOOST_PP_CAT(PointCloud,ELEM)::ConstPtr
typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARECLOUDVARIANT, ~, POINTTYPES) > cloud_variant_t;

struct PointCloud {

  struct holder_base {
    virtual cloud_variant_t make_variant() = 0;
  };

  template <typename T>
  struct holder : holder_base
  {
    T t;
    holder(T t_) : t(t_) { }

    cloud_variant_t make_variant()
    {
      return cloud_variant_t(t);
    }
  };

  boost::shared_ptr<holder_base> held;

  template <typename T>
  PointCloud(T t_)
  {
    held.reset(new holder<T>(t_));
  }

  PointCloud()
  {}

  cloud_variant_t make_variant()
  {
    return held->make_variant();
  }
};

// hacky pcl workaround for private PointCloud
template <typename Filter, typename PointType>
struct filter_takes_point_trait : boost::false_type {};

template <template <class> class Filter, typename PointType>
struct filter_takes_point_trait<Filter<PointType>, boost::shared_ptr<const pcl::PointCloud<PointType> > > : boost::true_type {};

typedef pcl::PointIndices indices_t;
typedef pcl::ModelCoefficients model_t;

using ecto::tendrils;

