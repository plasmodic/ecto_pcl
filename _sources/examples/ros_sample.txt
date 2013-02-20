Using ecto_pcl with ROS
=====================================

The ecto_pcl package includes ROS bindings for publishing and subscribing to
ROS topics with point clouds.

The syntax for a cloud subscriber is a bit involved as we must first subscribe
to a topic using an ecto_pcl_ros cell, and then convert the output of that cell
from a sensor_msgs::PointCloud into a pcl::PointCloud. We must be explicit about
the format of the generated cloud when doing this conversion, using one of the
defined cloud formats such as `ecto_pcl.XYZRGB` or `ecto_pcl.XYZ`.

:download:`ros_sample.py`

.. warning:: Temporarily disabled pending decisions on wtf to do about ROS

::

  .. literalinclude:: ros_sample.py

  .. ectoplot:: ros_sample.py plasm

