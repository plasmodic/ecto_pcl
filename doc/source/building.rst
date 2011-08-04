Building
========

.. highlight:: ectosh

With the pcl included in ROS electric
-------------------------------------

ecto_pcl by default builds against the ROS installation that it finds
via your environment (this is typically set by the scripts
``/opt/ros/<version>/setup.sh``).  When ROS is successfully found the
configuration looks something like this::

  % . /opt/ros/unstable/setup.zsh 
  % echo $ROS_ROOT
  /opt/ros/unstable/ros
  % cmake ../ecto_kitchen 
  -- The C compiler identification is GNU
  -- The CXX compiler identification is GNU
  -- Check for working C compiler: /home/troy/bin/gcc
    [etc]
  -- -~v/^\v~- pcl
  -- Attempting to find PCL via ROS environment because ECTO_PCL_STANDALONE is OFF.
  -- Boost version: 1.40.0
  -- Found the following Boost libraries:
  --   signals
  --   thread

Against standalone PCL
----------------------

If you want to build against a standalone PCL (e.g. PCL 1.1 as
available from standard ubuntu packages), pass
``-DECTO_PCL_STANDALONE=TRUE`` on the commandline or otherwise set
``ECTO_PCL_STANDALONE`` in your cmake cache.  Looks like this::

  % echo $ROS_ROOT

  % cmake ../ecto_kitchen
  -- -~v/^\v~- pcl
  -- + Setting default value of ECTO_PCL_STANDALONE to ON
  -- + Because ROS_ROOT is not set in your environment.
  -- + Searching for standalone PCL because ECTO_PCL_STANDALONE is ON
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
  
Against BOTH
------------

This is potentially dangerous, (see above) due to ODR violations from
the copying of point cloude message types, but what can you do.
Source your ros environment *and* set ``ECTO_PCL_STANDALONE`` to ON::
  
  % echo $ROS_ROOT
  /opt/ros/unstable/ros
  % cmake -DECTO_PCL_STANDALONE=ON ../ecto_kitchen 
  -- The C compiler identification is GNU
  ...
  
The output is basically the same as the standalone case. 
