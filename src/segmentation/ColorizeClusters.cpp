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
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointField.h>

namespace ecto {
  namespace pcl {

    struct ColorizeClusters
    {
      static void declare_params(tendrils& params)
      {
        params.declare<int> ("max_clusters", "Maximum number of clusters to output in the cloud.", 100);
        params.declare<float> ("saturation", "HSV Saturation of cloud colors on [0, 1]", 0.8);
        params.declare<float> ("value", "Value (brightness) of cloud colors on [0, 1]", 1.0);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        inputs.declare<Clusters> ("clusters", "Indices of clusters.");
        outputs.declare<PointCloud> ("output", "Colorized clusters as a single cloud.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        max_clusters_ = params["max_clusters"];

        clusters_ = inputs["clusters"];
        output_ = outputs["output"];

        saturation_ = params["saturation"];
        value_ = params["value"];
      }

      // see
      // http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
      // for points on a dark background you want somewhat lightened
      // colors generally... back off the saturation (s)
      static void hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
      {
        float c = v * s;
        float hprime = h/60.0;
        float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

        r = g = b = 0;

        if (hprime < 1) {
          r = c; g = x;
        } else if (hprime < 2) {
          r = x; g = c;
        } else if (hprime < 3) {
          g = c; b = x;
        } else if (hprime < 4) {
          g = x; b = c;
        } else if (hprime < 5) {
          r = x; b = c;
        } else if (hprime < 6) {
          r = c; b = x;
        }

        float m = v - c;
        r += m; g+=m; b+=m;
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        // initialize outputs and filter
        typename ::pcl::PointCloud<Point>::Ptr output(new typename ::pcl::PointCloud<Point>);
        ::pcl::ExtractIndices<Point> filter;
        filter.setInputCloud(input);
        output->header = input->header;

        // Extract location of rgb (similar to pcl::PackedRGBComparison<T>)
        std::vector<sensor_msgs::PointField> fields;
        ::pcl::getFields(*input, fields);
        size_t idx;
        for (idx = 0; idx < fields.size(); idx++)
        {
            if ( fields[idx].name == "rgb" || fields[idx].name == "rgba" )
                break;
        }
        if (idx == fields.size())
        {
            throw std::runtime_error("[ColorizeClouds] requires an rgb or rgba field.");
            return -1;
        }

        for (size_t i = 0; i < clusters_->size(); i++)
        {
            ::pcl::PointCloud<Point> cloud;
            // extract indices into a cloud
            filter.setIndices( ::pcl::PointIndicesPtr( new ::pcl::PointIndices ((*clusters_)[i])) );
            filter.filter(cloud);

            float hue = (360.0 / clusters_->size()) * i;

            float r, g, b;
            hsv2rgb(hue, *saturation_, *value_, r, g, b);

            // colorize cloud
            for (size_t j = 0; j < cloud.points.size(); j++)
            {
                Point &p = cloud.points[j];
                unsigned char* pt_rgb = (unsigned char*) &p;
                pt_rgb += fields[idx].offset;
                (*pt_rgb) = (unsigned char) (r * 255);
                (*(pt_rgb+1)) = (unsigned char) (g * 255);
                (*(pt_rgb+2)) = (unsigned char) (b * 255);
            }
            // append
            cloud.header = input->header;
            *output += cloud;
        }

        *output_ = xyz_cloud_variant_t(output);
        return OK;
      }

      spore<float> saturation_, value_;
      spore<int> max_clusters_;
      spore<Clusters> clusters_;
      spore<PointCloud> output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::ColorizeClusters>,
          "ColorizeClusters", "Concatenate clusters and colr each cluster differently.");

