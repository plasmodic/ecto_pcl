#pragma once

#include <ecto/ecto.hpp>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

typedef pcl::PointCloud<pcl::PointXYZ>    PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::Normal>      PointCloudNormal;

typedef boost::variant< PointCloudXYZ::ConstPtr, PointCloudXYZRGB::ConstPtr, PointCloudNormal::ConstPtr > cloud_variant_t;

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

//typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
//typedef pcl::PointCloud<pcl::Normal> normals_t;
typedef pcl::PointIndices indices_t;
typedef pcl::ModelCoefficients model_t;

using ecto::tendrils;

