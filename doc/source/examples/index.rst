Examples are found in the ``samples`` subdirectory of ``ecto_pcl``.

Examples
========

Currently the only difference between ROS-based and standalone
ecto_pcl is that the ecto_pcl.KinectGrabber does not exist in the
version built against ROS (we're working on resurrecting it).  The
following few examples show how to use the viewer, although any of the
ROS examples could be used in standalone by (replacing the message
grabbers with a KinectGrabber, which ahem doesn't exist) and
PointCloud publishers with a CloudViewer, then e.g. rviz could be used
for visualization.

.. toctree::
   :maxdepth: 1

   pcd_capture
   pcd_display
   pass_it_on
   tabletop_segmentation
   colorize_clusters


