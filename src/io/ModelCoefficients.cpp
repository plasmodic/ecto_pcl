#include "ecto_pcl.hpp"

namespace ecto {
  namespace pcl {

    struct PlaneCoefficientsFromFloat
    {
      static void declare_params(tendrils& params) { }
      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare< std::vector<float> > ("input", "Model coefficients as floats.");
        outputs.declare< ModelCoefficients::ConstPtr > ("output", "Model coefficients for use with ecto_pcl.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        input_ = inputs["input"];
        output_ = outputs["output"];
      }
      
      int process(const tendrils& inputs, const tendrils& outputs)
      {
        ModelCoefficients::Ptr model ( new ModelCoefficients() );
        model->values = *input_;
        *output_ = model;
        return OK;
      }

      spore< std::vector<float> > input_;
      spore< ModelCoefficients::ConstPtr > output_;
    };

  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::PlaneCoefficientsFromFloat,
          "PlaneCoefficientsFromFloat", "Convert vector of floats to model coefficients.");

