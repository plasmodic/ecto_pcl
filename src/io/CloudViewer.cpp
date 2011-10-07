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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/variant/get.hpp>

namespace ecto
{
  namespace pcl
  {
    using ::pcl::visualization::PCLVisualizer;
    struct CloudViewer
    {
      CloudViewer()
          :
            quit(false)
      {
      }

      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("window_name", "The window name", "cloud viewer");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud>("input", "The cloud to view");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        params["window_name"] >> window_name;
      }
      void
      run()
      {
        quit = false;
        viewer_.reset(new PCLVisualizer(window_name));
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(0.25);
        viewer_->initCameraParameters();

        while (!viewer_->wasStopped() && !boost::this_thread::interruption_requested())
        {
          {
            boost::mutex::scoped_try_lock lock(mtx);
            if (lock)
            {
              signal_();
              jobs_.clear(); //disconnects all the slots.
            }
          }
          viewer_->spinOnce(20);
        }
        quit = true;
      }

      struct show_dispatch: boost::static_visitor<>
      {
        show_dispatch(boost::shared_ptr<PCLVisualizer> viewer, const std::string& key)
            :
              viewer(viewer),
              key(key)
        {
        }

        //http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
        template<typename Point>
        void
        operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud) const
        {
          if (!viewer->updatePointCloud<Point>(cloud, key))
          {
            viewer->addPointCloud<Point>(cloud, key);
          }
        }
        void
        operator()(boost::shared_ptr<const CloudPOINTXYZRGB>& cloud) const
        {
          ::pcl::visualization::PointCloudColorHandlerRGBField<CloudPOINTXYZRGB::PointType> rgb(cloud);
          if (!viewer->updatePointCloud(cloud, rgb, key))
          {
            viewer->addPointCloud(cloud, rgb, key);
          }
        }
        void
        operator()(boost::shared_ptr<const CloudPOINTXYZRGBNORMAL>& cloud) const
        {
          ::pcl::visualization::PointCloudColorHandlerRGBField<CloudPOINTXYZRGBNORMAL::PointType> rgb(cloud);
//          ::pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<CloudPOINTXYZRGBNORMAL::PointType> normals(
//              cloud);

          if (!viewer->updatePointCloud(cloud, rgb, key))
          {
            viewer->addPointCloud(cloud, rgb, key);
          }
//          viewer->updatePointCloud(cloud,normals,key);
        }

        boost::shared_ptr<PCLVisualizer> viewer;
        std::string key;
      };
      struct show_dispatch_runner
      {
        show_dispatch_runner(const show_dispatch& dispatch, const xyz_cloud_variant_t& varient)
            :
              dispatch(dispatch),
              varient(varient)
        {
        }
        void
        operator()()
        {
          boost::apply_visitor(dispatch, varient);
        }
        show_dispatch dispatch;
        xyz_cloud_variant_t varient;
      };

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        if (quit)
        {
          runner_thread_->join();
          return ecto::QUIT;
        }
        if (!runner_thread_)
        {
          runner_thread_.reset(new boost::thread(boost::bind(&CloudViewer::run, this)));
        }
        while (!viewer_)
        {
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }

        {
          boost::mutex::scoped_lock lock(mtx);
          PointCloud cloud = inputs.get<PointCloud>("input");
          xyz_cloud_variant_t varient = cloud.make_variant();
          show_dispatch dispatch(viewer_, "main cloud");
          boost::shared_ptr<boost::signals2::scoped_connection> c(new boost::signals2::scoped_connection);
          *c = signal_.connect(show_dispatch_runner(dispatch, varient));
          jobs_.push_back(c);
        }

        return 0;
      }

      ~CloudViewer()
      {
        if (runner_thread_)
        {
          runner_thread_->interrupt();
          runner_thread_->join();
        }
      }
      std::string window_name;
      boost::shared_ptr<PCLVisualizer> viewer_;
      boost::shared_ptr<boost::thread> runner_thread_;
      boost::signals2::signal<void
      (void)> signal_;
      std::vector<boost::shared_ptr<boost::signals2::scoped_connection> > jobs_;
      boost::mutex mtx;
      bool quit;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::CloudViewer, "CloudViewer", "Viewer of clouds");

