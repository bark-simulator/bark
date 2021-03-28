
try:
    import debug_settings
except:
    pass

import unittest
import pickle
import numpy as np

# BARK 
from bark.core.world.renderer import *
from bark.core.geometry import *

class RendererTests(unittest.TestCase):
  def test_renderer(self):
    renderer = Renderer()
    renderer.Clear()
  
  def test_line2d_primitive(self):
    renderer = Renderer()
    renderer.Clear()
    l = Line2d()
    l.AddPoint(p)
    l.AddPoint(Point2d(10,4))
    l.AddPoint(Point2d(1.555555, 1.244222))
      
if __name__ == '__main__':
  unittest.main()