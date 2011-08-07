

template <typename T>
cv::Mat my_func(pcl::PointCloud<T> cloud)
{

}

#define DECLARE_MYFUNC(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) my_func< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARE_MYFUNC, ~, ECTO_XYZ_POINT_TYPES) > my_func_variant_t;



struct MyCell
{
  /* used to create a surface */
  template <template <class> class SurfacePolicy>
  struct make_surface_variant : boost::static_visitor<surface_variant_t>
  {
    template <typename CloudType >
    surface_variant_t operator()(const CloudType& p) const
    {
      return surface_variant_t(SurfacePolicy<typename CloudType::element_type::PointType>());
    }
  };

  /* dispatch to handle process */
  struct surface_dispatch : boost::static_visitor<xyz_cloud_variant_t>
  {
    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t operator()(Surface& f, CloudType& i) const
    {
      return impl(f, i, pcl_takes_point_trait<Surface, CloudType>());
    }

    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t impl(Surface& f, boost::shared_ptr<const CloudType>& i, boost::true_type) const
    {
      CloudType o;
      f.setInputCloud(i);
      f.reconstruct(o);
      return xyz_cloud_variant_t(o.makeShared());
    }

    template <typename Surface, typename CloudType>
    xyz_cloud_variant_t impl(Surface& f, CloudType& i, boost::false_type) const
    {
      throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
    }
  };

  static void declare_params(tendrils& params)
  {
    // no params
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<PointCloud> ("input", "The cloud to filter");
    outputs.declare<cv::Mat> ("output", "...");
  }

  MyCell() : configured_(false) {}

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    // set in/out.
    input_ = inputs["input"];
    output_ = outputs["output"];
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    xyz_cloud_variant_t cvar = input_->make_variant();
    if(!configured_){
      impl_ = boost::apply_visitor(make_my_func_variant<my_func>(), cvar);
      configured_ = true;
    }
    *output_ = boost::apply_visitor(surface_dispatch(), impl_, cvar);
    return 0;
  }
  
  bool configured_;
  my_cell_variant_t impl_;
  ecto::spore<PointCloud> input_;
  ecto::spore<cv::Mat> output_;

};

ECTO_CELL(ecto_pcl, MyCell, "MyCell", "..");
