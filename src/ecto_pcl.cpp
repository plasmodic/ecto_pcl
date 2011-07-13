#include <boost/python.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <ecto/ecto.hpp>

/* enumerations and values to be wrapped */
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/tree_types.h>

namespace bp = boost::python;

#define ENUMVAL(r, data, ELEM)                              \
  .value(BOOST_PP_STRINGIZE(BOOST_PP_CAT(SACMODEL_, ELEM)), \
         BOOST_PP_CAT(pcl::SACMODEL_, ELEM))

#define MODELTYPES                              \
  (PLANE)                                       \
  (LINE)                                        \
  (CIRCLE2D)                                    \
  (CIRCLE3D)                                    \
  (SPHERE)                                      \
  (CYLINDER)                                    \
  (CONE)                                        \
  (TORUS)                                       \
  (PARALLEL_LINE)                               \
  (PERPENDICULAR_PLANE)                         \
  (PARALLEL_LINES)                              \
  (NORMAL_PLANE)                                \
  (REGISTRATION)                                \
  (PARALLEL_PLANE)                              \
  (NORMAL_PARALLEL_PLANE)

ECTO_DEFINE_MODULE(ecto_pcl)
{ 
  bp::enum_<pcl::SacModel>("SacModel")
    BOOST_PP_SEQ_FOR_EACH(ENUMVAL, ~, MODELTYPES)
    .export_values()
    ;

  bp::scope().attr("KDTREE_FLANN") = pcl::KDTREE_FLANN;
  bp::scope().attr("KDTREE_ORGANIZED_INDEX") = pcl::KDTREE_ORGANIZED_INDEX;

}


