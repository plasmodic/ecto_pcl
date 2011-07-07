#!/usr/bin/env python

import ecto, ecto_pcl

print "sacmodel plane=", ecto_pcl.SacModel.SACMODEL_PLANE
print "sacmodel line=", ecto_pcl.SacModel.SACMODEL_LINE

ecto_pcl.showmemodel(ecto_pcl.SacModel.SACMODEL_LINE)

print "that should have shown 1, or", int(ecto_pcl.SacModel.SACMODEL_LINE)



