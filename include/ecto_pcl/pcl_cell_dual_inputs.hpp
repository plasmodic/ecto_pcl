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
    struct PclCellDualInputs
    {
      static void declare_params(ecto::tendrils& params)
      {
        CellType::declare_params(params);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<PointCloud> ("input", "Input cloud.").required(true);
        inputs.declare<PointCloud> (CellType::SecondInputName, CellType::SecondInputDescription).required(true);
        CellType::declare_io(params, inputs, outputs);
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input1_ = inputs["input"];
        input2_ = inputs[CellType::SecondInputName];
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
        void operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud1,
                        boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud2) const
        {   
          ft.process(inputs, outputs, cloud1, cloud2);
        }

        template <typename Point, typename Point2>
        void operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud1,
                        boost::shared_ptr<const ::pcl::PointCloud<Point2> >& cloud2) const
        {   
          throw std::runtime_error("PclCellDualInput: clouds are not the same type!");
        }
      };

      int process(const tendrils& inputs, const tendrils& outputs)
      {
        xyz_cloud_variant_t cloud1 = input1_->make_variant();
        xyz_cloud_variant_t cloud2 = input2_->make_variant();
        boost::apply_visitor(filter_dispatch(impl_, inputs, outputs), cloud1, cloud2);
        return 0;
      }

      ecto::spore<PointCloud> input1_;
      ecto::spore<PointCloud> input2_;
      CellType impl_;
    };

  }
}

