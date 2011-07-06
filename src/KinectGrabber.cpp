#include <ecto/ecto.hpp>

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

using ecto::tendrils;

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
class SimpleKinectGrabber
{
public:
  SimpleKinectGrabber() 
    : thread_(boost::ref(*this))
  { }

  ~SimpleKinectGrabber() { }

  void operator ()()
  {

    boost::scoped_ptr<pcl::Grabber> interface(new pcl::OpenNIGrabber());

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> point_cloud_cb =
        boost::bind(&SimpleKinectGrabber::cloud_cb_, this, _1);

    boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&,
                         const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> image_depth_cb =
        boost::bind(&SimpleKinectGrabber::image_depth_cb_, this, _1, _2, _3);

    //interface->
    boost::signals2::connection c = interface->registerCallback(point_cloud_cb);

    interface->start();

    while (!thread_.interruption_requested())
      {
        boost::thread::yield();
      }

    c.disconnect();
    std::cerr << "Stopping" << std::endl;

    interface->stop();
  }

  /**
   * \brief don't hang on to this cloud!! or it won't get updated.
   */
  cloud_t::ConstPtr getLatestCloud()
  {
    boost::mutex::scoped_lock lock(mutex_);
    cloud_t::ConstPtr p = cloud_;
    cloud_.reset();
    return p;
  }
  //depth callback.
  void image_depth_cb_(const boost::shared_ptr<openni_wrapper::Image>&,
                       const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)
  {

  }
  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cloud_ = cloud;
  }

  boost::mutex mutex_;
  cloud_t::ConstPtr cloud_;
  boost::thread thread_;

};
ECTO_CELL(ecto_pcl, SimpleKinectGrabber, "SimpleKinectGrabber", "Simple kinect grabber");


struct KinectGrabber
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    outputs.declare<cloud_t::ConstPtr> ("output", "An rgb xyz point cloud from the kinect");
  }

  int process(const tendrils& /*inputs*/, tendrils& outputs)
  {
    cloud_t::ConstPtr p;
    while (!p)
      {
        p = grabber_.getLatestCloud();
        boost::thread::yield();
      }
    outputs.get<cloud_t::ConstPtr> ("output") = p;
    return 0;
  }

  SimpleKinectGrabber grabber_;
};

ECTO_CELL(ecto_pcl, KinectGrabber, "KinectGrabber", "Grabber from kinect");

