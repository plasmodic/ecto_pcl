#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#undef  BOOST_PARAMETER_MAX_ARITY
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#if PCL_VERSION_COMPARE(>=,1,7,0)
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif

typedef sensor_msgs::PointCloud2::ConstPtr MsgT;

namespace ecto
{
  namespace pcl_ros
  {
    template<typename PointT>
    typename ::pcl::PointCloud<PointT>::ConstPtr
    convert(MsgT msg)
    {
      typedef typename ::pcl::PointCloud<PointT> CloudT;
      typedef typename CloudT::Ptr PtrT;
      PtrT p(new CloudT);
#if PCL_VERSION_COMPARE(<,1,7,0)
      ::pcl::fromROSMsg(*msg, *p);
#else
      ::pcl::PCLPointCloud2 pcd_tmp;
      pcl_conversions::toPCL(*msg, pcd_tmp);
      ::pcl::fromPCLPointCloud2(pcd_tmp, *p);
#endif
      return p;
    }

    /* dispatch to handle process */
    struct to_message: boost::static_visitor<MsgT>
    {
      template<typename CloudType>
      MsgT
      operator()(CloudType& i) const
      {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
#if PCL_VERSION_COMPARE(<,1,7,0)
        ::pcl::toROSMsg(*i, *msg);
#else
        ::pcl::PCLPointCloud2 pcd_tmp;
        ::pcl::toPCLPointCloud2(*i, pcd_tmp);
        pcl_conversions::fromPCL(pcd_tmp, *msg);
#endif
        return msg;
      }
    };

    struct Message2PointCloud
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<int>("format", "Format of cloud to grab. Choices are: XYZ, XYZRGB (default)", ecto::pcl::FORMAT_XYZRGB);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<MsgT>("input", "An ROS point cloud message.");
        outputs.declare<ecto::pcl::PointCloud>("output", "An XYZ/XYZRGB point cloud from the kinect");
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
            *output_ = convert< ::pcl::PointXYZ >(*input_);
            break;
          case ecto::pcl::FORMAT_XYZRGB:
            *output_ = convert< ::pcl::PointXYZRGB >(*input_);
            break;
          case ecto::pcl::FORMAT_XYZI:
            *output_ = convert< ::pcl::PointXYZI >(*input_);
            break;
          case ecto::pcl::FORMAT_XYZRGBA:
            *output_ = convert< ::pcl::PointXYZRGBA >(*input_);
            break;
          default:
            throw std::runtime_error("Unsupported point cloud type.");
        }
        return ecto::OK;
      }
      ecto::spore<int> format_;
      ecto::spore<MsgT> input_;
      ecto::spore<ecto::pcl::PointCloud> output_;
    };

    struct PointCloud2Message
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<int>("format", "Format of cloud to grab. Choices are: XYZ, XYZRGB (default)", ecto::pcl::FORMAT_XYZRGB);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<ecto::pcl::PointCloud>("input", "An ROS point cloud message.");
        outputs.declare<MsgT>("output", "An XYZ/XYZRGB point cloud from the kinect");
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
        ecto::pcl::xyz_cloud_variant_t v = input_->make_variant();
        *output_ = boost::apply_visitor(to_message(),v);
        return ecto::OK;
      }
      ecto::spore<int> format_;
      ecto::spore<ecto::pcl::PointCloud> input_;
      ecto::spore<MsgT> output_;
    };
  }
}

ECTO_CELL(ecto_pcl_ros, ecto::pcl_ros::Message2PointCloud,
          "Message2PointCloud", "Take a PointCloud Message and converts to pcl type.");
ECTO_CELL(ecto_pcl_ros, ecto::pcl_ros::PointCloud2Message,
          "PointCloud2Message", "Take a pcl type and converts to PointCloud Message.");

