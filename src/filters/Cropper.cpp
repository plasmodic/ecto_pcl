/*
 * Cropper.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: tcavallari
 */

#include <iostream>
#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

namespace ecto
{
	namespace pcl
	{
		struct Cropper
		{
			static void
			declare_params(ecto::tendrils& params)
			{
				params.declare(&Cropper::enabled_, "crop_enabled", "If the cropper cell is enabled", true);

				params.declare(&Cropper::x_min_, "x_min", "The minimum x value (in the camera reference frame)", -std::numeric_limits<float>::max());
				params.declare(&Cropper::x_max_, "x_max", "The maximum x value (in the camera reference frame)", std::numeric_limits<float>::max());
				params.declare(&Cropper::y_min_, "y_min", "The minimum y value (in the camera reference frame)", -std::numeric_limits<float>::max());
				params.declare(&Cropper::y_max_, "y_max", "The maximum y value (in the camera reference frame)", std::numeric_limits<float>::max());
				params.declare(&Cropper::z_min_, "z_min", "The minimum z value (in the camera reference frame)", -std::numeric_limits<float>::max());
				params.declare(&Cropper::z_max_, "z_max", "The maximum z value (in the camera reference frame)", std::numeric_limits<float>::max());
			}

			static void
			declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
			{
			  // input already declared by the ectoPcl cell

			  outputs.declare(&Cropper::cloud_out_, "output", "The cropped cloud.");
			}

			void
			configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
			{
			}

			template<typename PointType>
			int	process(const ecto::tendrils& inputs, const ecto::tendrils& outputs, boost::shared_ptr<const ::pcl::PointCloud<PointType> >& input)
			{
				if(!(*enabled_))
				{
					*cloud_out_ = xyz_cloud_variant_t(input);
					return ecto::OK;
				}

				::pcl::PassThrough<PointType> pass_filter;

				typename ::pcl::PointCloud<PointType>::Ptr z_cloud_filtered_ptr(new ::pcl::PointCloud<PointType>()),
						y_cloud_filtered_ptr(new ::pcl::PointCloud<PointType>()),
						x_cloud_filtered_ptr(new ::pcl::PointCloud<PointType>());

				pass_filter.setInputCloud(input);
				pass_filter.setFilterFieldName("z");
				pass_filter.setFilterLimits(*z_min_, *z_max_);
				pass_filter.filter(*z_cloud_filtered_ptr);

				pass_filter.setInputCloud(z_cloud_filtered_ptr);
				pass_filter.setFilterFieldName("y");
				pass_filter.setFilterLimits(*y_min_, *y_max_);
				pass_filter.filter(*y_cloud_filtered_ptr);

				pass_filter.setInputCloud(y_cloud_filtered_ptr);
				pass_filter.setFilterFieldName("x");
				pass_filter.setFilterLimits(*x_min_, *x_max_);
				pass_filter.filter(*x_cloud_filtered_ptr);

				*cloud_out_ = xyz_cloud_variant_t(x_cloud_filtered_ptr);

				return ecto::OK;
			}

			// Params
			ecto::spore<bool> enabled_;
			ecto::spore<float> x_min_;
			ecto::spore<float> x_max_;
			ecto::spore<float> y_min_;
			ecto::spore<float> y_max_;
			ecto::spore<float> z_min_;
			ecto::spore<float> z_max_;


			// I/O
			ecto::spore<PointCloud> cloud_out_;
		};
	}
}

ECTO_CELL(ecto_pcl, ecto::pcl::PclCell<ecto::pcl::Cropper>, "Cropper", "Crops and keeps organized a given point cloud.");

