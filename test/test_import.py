#!/usr/bin/env python
from __future__ import print_function
import ecto_pcl
import unittest

class TestPcl(unittest.TestCase):

    def test_import(self):
        print(ecto_pcl)

if __name__ == '__main__':
    import rosunit
    rosunit.rosrun('ecto_pcl', 'test_import', TestPcl)
