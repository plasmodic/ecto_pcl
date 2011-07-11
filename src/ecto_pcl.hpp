#pragma once

#include <ecto/ecto.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
typedef pcl::PointCloud<pcl::Normal> normals_t;
typedef pcl::PointIndices indices_t;
typedef pcl::ModelCoefficients model_t;

using ecto::tendrils;

