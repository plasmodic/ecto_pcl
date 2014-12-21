0.4.2 (2014-12-21)
------------------
* Use the new header with PCL 1.7.0+
* Always find pcl_conversions
  Should be okay since it's a package dependency.
* Find pcl_conversions if building against PCL 1.7.0+
  It gets included depending on the PCL version in src/ros/pcl_bridge.cpp
* fix sample with newer API
* Contributors: Vincent Rabaud, pgorczak

0.4.1 (2014-10-15)
------------------
* Merge pull request `#34 <https://github.com/plasmodic/ecto_pcl/issues/34>`_ from v4hn/pass-through-indices
  Add PassThrough for index extraction
* Merge pull request `#33 <https://github.com/plasmodic/ecto_pcl/issues/33>`_ from v4hn/euclidean-cluster-extraction-indices
  Allow EuclideanClusterExtraction to work on input indices
* Add PassThrough for index extraction
  This module behaves similarly to PassThrough with two changes:
  - It also accepts an optional set of input indices of the input cloud
  It's probably not good to add this to PassThrough as well, because it breaks whatever[:] >> passthrough[:]
  - It produces PointIndices of the original cloud instead of a new cloud
* Allow EuclideanClusterExtraction to work on input indices
  This allows for a pipeline which outputs Clusters as sets of indices
  of the _original_ (organized) input cloud from the depth image camera.
  That way 2d bounding boxes in the input image can be more efficiently computed.
* Merge pull request `#32 <https://github.com/plasmodic/ecto_pcl/issues/32>`_ from v4hn/pcl-cell-return-values
  pcl_cell: don't ignore the return value of the wrapped cell
* pcl_cell: don't ignore the return value of the wrapped cell
  This makes sure ecto actually stops processing in case on of the cells
  produces an error and returns something else than ecto::OK.
* Merge pull request `#31 <https://github.com/plasmodic/ecto_pcl/issues/31>`_ from v4hn/convex-hull-explicit-dimensionality
  ConvexHull: add parameter to explicitly set dimensionality of the data/hull
* ConvexHull: add parameter to explicitly set dimensionality of the data/hull
  I had qhull 2012 via pcl 1.7.2 complaining over here that the input is,
  indeed, _not_ three dimensional when computing the hull of a detected table.
  Turned out pcl is always told to reconstruct a 3d-hull.
* Contributors: Michael Görner, Vincent Rabaud, v4hn

0.4.0 (2014-04-04)
------------------
* depend on pcl_conversions which is lighter
* drop Fuerte support
* Contributors: Vincent Rabaud

0.3.14 (2013-08-28 19:12:21 -0800)
----------------------------------
- fix dependencies with PCL
