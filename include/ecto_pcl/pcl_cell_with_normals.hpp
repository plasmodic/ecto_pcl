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

#pragma once

namespace ecto {
  namespace pcl {

    template <typename CellType>
    struct PclCellWithNormals
    {
      static void declare_params(ecto::tendrils& params)
      {
        CellType::declare_params(params);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "Input cloud.").required(true);
        inputs.declare<FeatureCloud> ("normals", "Normals to use.").required(true);
        CellType::declare_io(params, inputs, outputs);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input_ = inputs["input"];
        normals_ = inputs["normals"];
        impl_.configure(params, inputs, outputs);
      }

      /* dispatch to handle process */
      struct filter_dispatch : boost::static_visitor<void>
      {
        CellType& ft;
        const tendrils& inputs;
        const tendrils& outputs;

        filter_dispatch(CellType& ft_, const tendrils& input_, const tendrils& output_) : ft(ft_), inputs(input_), outputs(output_) { }

        template <typename Point>
        void operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& input, 
                        boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals) const
        {   
          ft.process(inputs, outputs, input, normals);
        }

        template <typename Point, typename Normal>
        void operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                        boost::shared_ptr<const ::pcl::PointCloud<Normal> >& normals) const
        {   
          throw std::runtime_error("PclCellWithNormals: normals must be of type pcl::Normal!");
        }
      };

      int process(const tendrils& inputs, const tendrils& outputs)
      {
        xyz_cloud_variant_t input = input_->make_variant();
        feature_cloud_variant_t normals = normals_->make_variant();
        boost::apply_visitor(filter_dispatch(impl_, inputs, outputs), input, normals);
        return 0;
      }

      ecto::spore<PointCloud> input_;
      ecto::spore<FeatureCloud> normals_;
      CellType impl_;
    };

  }
}

