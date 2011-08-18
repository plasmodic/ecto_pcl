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

#include "ecto_pcl.hpp"
#include "pcl_cell.hpp"
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointField.h>

#include <ros/ros.h>

struct ColorizeClusters
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int> ("max_clusters", "Maximum number of clusters to output in the cloud.", 100);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare<cluster_t> ("clusters", "Indices of clusters.");
    outputs.declare<PointCloud> ("output", "Colorized clusters as a single cloud.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    max_clusters_ = params["max_clusters"];

    clusters_ = inputs["clusters"];
    output_ = outputs["output"];
  }

  template <typename Point>
  int process(const tendrils& inputs, const tendrils& outputs, 
              boost::shared_ptr<const pcl::PointCloud<Point> >& input)
  {
    // initialize outputs and filter
    pcl::PointCloud<Point> output;
    pcl::ExtractIndices<Point> filter;
    filter.setInputCloud(input);
    output.header = input->header;

    // Extract location of rgb (similar to pcl::PackedRGBComparison<T>)
    std::vector<sensor_msgs::PointField> fields;
    pcl::getFields(*input, fields);
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

    // initialize colors
    int r = 0;
    int g = 0;
    int b = 0;

    for (size_t i = 0; i < clusters_->size(); i++)
    {
        pcl::PointCloud<Point> cloud;
        // extract indices into a cloud
        filter.setIndices( pcl::PointIndicesPtr( new pcl::PointIndices ((*clusters_)[i])) );
        filter.filter(cloud);
        // determine color
        if(i%3 == 0){
            r = (r+128)%256;
        }else if(i%3 == 1){
            g = (g+128)%256;
        }else if(i%3 == 2){
            b = (b+128)%256;
        }
        // colorize cloud
        for (size_t j = 0; j < cloud.points.size(); j++)
        {
            Point &p = cloud.points[j];
            unsigned char* pt_rgb = (unsigned char*) &p;
            pt_rgb += fields[idx].offset;
            (*pt_rgb) = (unsigned char) r;
            (*(pt_rgb+1)) = (unsigned char) g;
            (*(pt_rgb+2)) = (unsigned char) b;
        }
        // append
        cloud.header = input->header;
        output += cloud;
    }
    ROS_INFO_STREAM("Colorize Clusters: clusters = " << clusters_->size());

    *output_ = xyz_cloud_variant_t(output.makeShared());
    return ecto::OK;
  }

  ecto::spore<int> max_clusters_;  
  ecto::spore<cluster_t> clusters_;
  ecto::spore<PointCloud> output_;
};

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ColorizeClusters>, "ColorizeClusters", "Concatenate clusters and colr each cluster differently.");

