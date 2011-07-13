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
  PointCloudXYZRGB::ConstPtr getLatestCloud()
  {
    boost::mutex::scoped_lock lock(datamutex_);

    while (!cloud_)
      cond_.wait(lock);

    PointCloudXYZRGB::ConstPtr p = cloud_;
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

  PointCloudXYZRGB::ConstPtr cloud_;
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

