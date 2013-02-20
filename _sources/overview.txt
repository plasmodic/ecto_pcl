Overview
==================

Common Data Types
-------------------------------------
If you have used PCL in C++, you'll notice that a point cloud is a templated type (pcl::PointCloud<PointT>). To handle this with our Python wrappers, we have defined: 

 - ``ecto::pcl::PointCloud`` - a data type that stores a pcl::PointCloud, hiding the PointT type. ecto::pcl::PointCloud will always have X, Y, and Z components and may also have things such as RGB, Alpha, etc. 
 - ``ecto::pcl::FeatureCloud`` - a data type that stores a pcl::PointCloud, hiding the PointT type. This is similar to ecto::pcl::PointCloud, however it is intended for clouds of features, such as Normals, VPFH, etc. 

There are additional common data types defined: 
  
 - ``ecto::pcl::Indices``
 - ``ecto::pcl::Clusters``
 - ``ecto::pcl::ModelCoefficients``

Creating New Cells
-------------------------------------
Each of the above data types is implemented with the help of boost variants. To ease the difficulty in using them, we have created a number of stubs for typical ecto_pcl cells: 

 - ``ecto::pcl::PclCell`` (pcl_cell.hpp) - for cells that take a single PointCloud input.
 - ``ecto::pcl::PclCellDualInputs`` (pcl_cell_dual_inputs.hpp) - for cells with two PointCloud inputs.
 - ``ecto::pcl::PclCellWithNormals`` (pcl_cell_with_normals.hpp) - for cells that need a PointCloud and Normals (a FeatureCloud). 

Using each of these is similar. You include the header file and declare your cell. The only difference from a typical ecto cell is that the process() callback will be templated on the point type and have extra parameters corresponding to the extracted pcl::PointClouds<PointT>:

.. literalinclude:: example_filter.cpp
  :language: cpp

The key aspects here are: 
 
 - Our cell is declared in the ECTO_CELL as ``ecto::pcl::PclCell<ExampleFilter>``.
 - We declare parameters and IO as usual.
 - ``ecto::pcl::PclCell`` already declares an input PointCloud called "input". 
 - Our process callback is templated on point type.
 - Our process callback has an extra parameter to a templated ``pcl::PointCloud`` (held in a shared pointer). 
 - To set the value of our output spore we have to use: ``ecto::pcl::xyz_cloud_variant_t`` (``boost_shared`` pointer to our cloud);
