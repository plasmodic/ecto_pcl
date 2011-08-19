Examples are found in the ``samples`` subdirectory of ``ecto_pcl``.

ROS based Examples
==================
The following example sshow how to use ``ecto_pcl`` with ROS.

.. toctree::
   :maxdepth: 1

   pass_it_on
   tabletop_segmentation
   colorize_clusters

Standalone Examples
===================

The only difference between ROS-based and standalone ecto_pcl is the
addition of several input/output cells, namely a CloudViewer (we're
working on resurrecting the KinectGrabber). The following few examples
show how to use the viewer, although any of the ROS examples could be
used in standalone by replacing the message grabbers with a
KinectGrabber and PointCloud publishers with a CloudViewer.

.. todo:: Examples of standalone kinectgrabber/cloudviewer


