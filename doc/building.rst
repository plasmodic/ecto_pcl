Building
========

.. highlight:: ectosh

PCL is simply found through a find_package(PCL) macro:

  % cmake ../
  -- -~v/^\v~- pcl
  -- + Because ROS_ROOT is not set in your environment.
  -- Found wxWidgets: TRUE 
  -- + Found PCL version 1.1.0
  -- *** You will STILL BUILD AN ECTO_PCL_ROS LIBRARY
  -- *** Which has ecto cell converters from sensor_msgs to point cloud
  -- *** That may be Dangerous, if your ecto_ros compiles against messages
  -- *** with the same name but different size, and both run in the same
  -- *** process simultaneously.
  -- *** See http://en.wikipedia.org/wiki/One_Definition_Rule
  -- Boost version: 1.40.0
  -- Found the following Boost libraries:
  --   signals
  --   thread

If not found you'll get the standardish cmake error message::

  -- + Searching for standalone PCL
  CMake Warning at pcl/CMakeLists.txt:38 (find_package):
    Could not find module FindPCL.cmake or a configuration file for package
    PCL.
  
    Adjust CMAKE_MODULE_PATH to find FindPCL.cmake or set PCL_DIR to the
    directory containing a CMake configuration file for PCL.  The file will
    have one of the following names:
  
      PCLConfig.cmake
      pcl-config.cmake
  
  
  
  **
  ** Disabling build of ecto_pcl due to missing dependency PCL
  **
