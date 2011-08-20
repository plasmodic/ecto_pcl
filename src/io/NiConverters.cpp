
#include "ecto_pcl.hpp"


#include <iostream>
#include <string>
using ecto::tendrils;

namespace ecto {
  namespace pcl {

      typedef std::vector<uint8_t> RgbData;
      typedef std::vector<uint16_t> DepthData;

      typedef boost::shared_ptr<RgbData> RgbDataPtr;
      typedef boost::shared_ptr<DepthData> DepthDataPtr;

      typedef boost::shared_ptr<const RgbData> RgbDataConstPtr;
      typedef boost::shared_ptr<const DepthData> DepthDataConstPtr;

      struct NiConverter
      {
        static void
        declare_io(const tendrils& params, tendrils& i, tendrils& o)
        {
          i.declare<int>("depth_width", "Depth frame width.");
          i.declare<int>("depth_height", "Depth frame height.");
          i.declare<int>("image_width", "Image frame width.");
          i.declare<int>("image_height", "Image frame height.");
          i.declare<int>("image_channels", "Number of image channels.");
          i.declare<DepthDataConstPtr>("depth_buffer");
          i.declare<RgbDataConstPtr>("image_buffer");

          //o.declare<cv::Mat>("image");
          //o.declare<cv::Mat>("depth");
          o.declare<ecto::pcl::PointCloud>("output");
        }

        void
        configure(const tendrils& p, const tendrils& i, const tendrils& o)
        {
          depth_height = i["depth_height"];
          depth_width = i["depth_width"];
          image_width = i["image_width"];
          image_height = i["image_height"];
          image_channels = i["image_channels"];
          image_buffer = i["image_buffer"];
          depth_buffer = i["depth_buffer"];
          //image = o["image"];
          //depth = o["depth"];
          output = o["output"];
        }

        int
        process(const tendrils&, const tendrils&)
        {
          if (*image_buffer && *depth_buffer)
          {
            ::pcl::PointCloud< ::pcl::PointXYZRGB > cloud;

            cloud.width = *depth_width;
            cloud.height = *depth_height;
            cloud.points.resize(cloud.width * cloud.height);
            
            //grab camera params
            float fx = 525; //K.at<float>(0, 0);
            float fy = 525; //K.at<float>(1, 1);
            float cx = 640/2.0 - .5; //K.at<float>(0, 2);
            float cy = 480/2.0 - .5; //K.at<float>(1, 2);
            
            const uint16_t* d;
            const uint8_t* c;
            d = (*depth_buffer)->data();
            c = (*image_buffer)->data();
            for (size_t v = 0; v < cloud.height; v++)
            { 
              for (size_t u = 0; u < cloud.width; u++)
              {
                uint8_t r = *(c++);
                uint8_t g = *(c++);
                uint8_t b = *(c++);
                ::pcl::PointXYZRGB p(r, g, b);

                uint16_t fpz = *(d++);            
                float z =  fpz / 1000.0f;
                p.x = (u - cx) * z / fx;
                p.y = (v - cy) * z / fy;
                p.z = z;            
                cloud(u,v) = p;
              }
            }

            *output = ecto::pcl::xyz_cloud_variant_t(cloud.makeShared());
          }
          return ecto::OK;
        }

        ecto::spore<int> depth_width, depth_height, image_width, image_height, image_channels;
        ecto::spore<DepthDataConstPtr> depth_buffer;
        ecto::spore<RgbDataConstPtr> image_buffer;

        //ecto::spore<cv::Mat> image, depth;
        ecto::spore<ecto::pcl::PointCloud> output;
    };

  }
}
ECTO_CELL(ecto_pcl, ecto::pcl::NiConverter, "NiConverter", "Read images from a directory.");
