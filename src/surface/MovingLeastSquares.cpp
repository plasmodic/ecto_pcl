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

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <boost/typeof/typeof.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/mls.h>

namespace ecto
{
  namespace pcl
  {
    template<typename PointT, unsigned>
    float*
    get_normal(PointT* pt)
    {
      return pt->normal;
    }

    template<typename PointT>
    float*
    get_normal(...)
    {
      //BOOST_STATIC_ASSERT(sizeof(PointT) == 0);
      return 0;
    }

    struct MovingLeastSquares
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare(&MovingLeastSquares::search_radius_, "search_radius", "Sphere to be considered a neighbor.", 0.03);
        p.declare(&MovingLeastSquares::polynomial_order_, "polynomial_order", "The polynomial order to fit.", 2);
      }

      static void
      declare_io(const tendrils& p, tendrils& i, tendrils& o)
      {
        o.declare(&ecto::pcl::PclCell<ecto::pcl::MovingLeastSquares>::output_, "output", "The smoothed cloud.");
      }

      void
      configure(const tendrils& /*p*/, const tendrils& /*i*/, const tendrils& /*o*/)
      {
      }

      template<typename Point>
      int
      process(const tendrils& /*inputs*/, const tendrils& /*outputs*/,
          boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        throw std::runtime_error("Not supported.");
        return ecto::OK;
      }

      int
      process(const tendrils& /*inputs*/, const tendrils& /*outputs*/,
          boost::shared_ptr<const ::pcl::PointCloud< ::pcl::PointXYZRGBNormal> >& input)
      {
        typedef ::pcl::PointXYZRGBNormal Point;
        typedef ::pcl::Normal Normal;
        typedef ::pcl::PointCloud<Point> Cloud;
        typedef ::pcl::PointCloud<Normal> Normals;

        typedef ::pcl::search::KdTree<Point> KdTree;
        typedef/*typename*/KdTree::Ptr KdTreePtr;
        typedef/*typename*/Cloud::Ptr CloudPtr;
#if PCL_VERSION_COMPARE(<,1,6,0)
        ::pcl::MovingLeastSquares<Point, Normal> mls;
#else
        ::pcl::MovingLeastSquares<Point, Point> mls;
#endif
        Normals::Ptr normals(new Normals);
        CloudPtr smoothed_cloud(new Cloud);
        // Create search tree*
        KdTreePtr mls_tree(new KdTree);
        mls.setSearchMethod(mls_tree);
        *output_ = xyz_cloud_variant_t(smoothed_cloud);
        if(input->size() < 1)
          return ecto::OK;
        // Set parameters
        mls.setInputCloud(input);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(*polynomial_order_);
        mls.setSearchMethod(mls_tree);
        mls.setSearchRadius(*search_radius_);

#if PCL_VERSION_COMPARE(<,1,6,0)
        mls.setOutputNormals(normals);
        // Reconstruct 
        mls.reconstruct(*smoothed_cloud);

        for (size_t i = 0, end = normals->size(); i < end; i++)
        {
          const float* n = normals->points[i].normal;
          Point& x = smoothed_cloud->points[i];
          float* n_r = x.normal;  
          float sign = n[0] * n_r[0] + n[1] * n_r[1] + n[2] * n_r[2] < 0 ? -1 : 1;
          n_r[0] = sign * n[0];
          n_r[1] = sign * n[1];
          n_r[2] = sign * n[2];
        }

#else
        mls.setComputeNormals(true);
        // Reconstruct 
        mls.process(*smoothed_cloud);
#endif

        *output_ = xyz_cloud_variant_t(smoothed_cloud);
        return ecto::OK;
      }
      spore<double> search_radius_;
      spore<int> polynomial_order_;
      spore<PointCloud> output_;
    }
    ;

  }
}

  ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::MovingLeastSquares>, "MovingLeastSquares",
            "Smooth a cloud with MLS.");

