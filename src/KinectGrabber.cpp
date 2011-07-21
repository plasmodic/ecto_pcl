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

#undef  BOOST_PARAMETER_MAX_ARITY
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <iostream>

class SimpleKinectGrabber
{
public:
  SimpleKinectGrabber() 
    : thread_(boost::ref(*this))
  { 
    runmutex_.lock(); // when this gets unlocked in the destructor, the thread unblocks
  }

  ~SimpleKinectGrabber() { }

  void stop() {
    runmutex_.unlock();
    thread_.join();
  }

  void operator ()()
  {
    boost::scoped_ptr<pcl::Grabber> interface(new pcl::OpenNIGrabber);

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> point_cloud_cb =
        boost::bind(&SimpleKinectGrabber::cloud_cb_, this, _1);

    boost::signals2::connection c = interface->registerCallback(point_cloud_cb);

    interface->start();

    runmutex_.lock(); // blocks until the destructor unlocks it
    runmutex_.unlock(); // unlock it again... why does this stop a crash?

    c.disconnect();

    interface->stop();
  }

  /**
   * \brief don't hang on to this cloud!! or it won't get updated.
   */
  CloudPOINTXYZRGB::ConstPtr getLatestCloud()
  {
    boost::mutex::scoped_lock lock(datamutex_);

    while (!cloud_)
      cond_.wait(lock);

    CloudPOINTXYZRGB::ConstPtr p = cloud_;
    cloud_.reset();
    return p;
  }

  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    boost::lock_guard<boost::mutex> lock(datamutex_);
    cloud_ = cloud;
    cond_.notify_one();
  }

  boost::mutex datamutex_, runmutex_;
  boost::condition_variable cond_;

  CloudPOINTXYZRGB::ConstPtr cloud_;
  boost::thread thread_;
};
ECTO_CELL(ecto_pcl, SimpleKinectGrabber, "SimpleKinectGrabber", "Simple kinect grabber");


struct KinectGrabber
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    outputs.declare<PointCloud> ("output", "An XYZRGB point cloud from the kinect");
  }

  int configure(const tendrils& parameters, tendrils& inputs, tendrils& outputs)
  {
    impl_.reset(new SimpleKinectGrabber);
  }

  int process(const tendrils& /*inputs*/, tendrils& outputs)
  {
    PointCloud p( impl_->getLatestCloud() );
    outputs.get<PointCloud> ("output") = p;
    return 0;
  }

  ~KinectGrabber() { impl_->stop(); }
  boost::scoped_ptr<SimpleKinectGrabber> impl_;
};

ECTO_CELL(ecto_pcl, KinectGrabber, "KinectGrabber", "Grabber from kinect");

