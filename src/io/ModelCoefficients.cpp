#include "ecto_pcl.hpp"

struct PlaneCoefficientsFromFloat
{
  static void declare_params(ecto::tendrils& params) { }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< std::vector<float> > ("input", "Model coefficients as floats.");
    outputs.declare< ecto::pcl::ModelCoefficients::ConstPtr > ("output", "Model coefficients for use with ecto_pcl.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    input_ = inputs["input"];
    output_ = outputs["output"];
  }
  
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    ecto::pcl::ModelCoefficients::Ptr model ( new ecto::pcl::ModelCoefficients() );
    model->values = *input_;
    *output_ = model;
    return ecto::OK;
  }

  ecto::spore< std::vector<float> > input_;
  ecto::spore< ecto::pcl::ModelCoefficients::ConstPtr > output_;
};

ECTO_CELL(ecto_pcl, PlaneCoefficientsFromFloat, "PlaneCoefficientsFromFloat", "Convert vector of floats to model coefficients.");

