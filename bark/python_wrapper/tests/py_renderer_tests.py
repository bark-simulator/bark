
try:
    import debug_settings
except:
    pass

import unittest

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
    l.AddPoint(Point2d(10, 4))
    l.AddPoint(Point2d(1.555555, 1.244222))
    
    line_primitive = RenderPrimitive(l)
    renderer.Add("line_prim_0", line_primitive)
    renderer.Add("line_prim_0", line_primitive)
    renderer.Add("line_prim_1", line_primitive)
    
    # asserts
    rp = renderer.primitives
    self.assertTrue(len(rp) == 2)
    self.assertTrue(
      (l.ToArray() == rp["line_prim_0"][0].object.ToArray()).all())
    self.assertTrue(
      (l.ToArray() == rp["line_prim_0"][1].object.ToArray()).all())
    self.assertTrue(
      (l.ToArray() == rp["line_prim_1"][0].object.ToArray()).all())
  
  
  def test_polygon_primitive(self):
    renderer = Renderer()
    renderer.Clear()
    polygon = Polygon2d(
      [0, 0, 0],
      [Point2d(-1,-1),
       Point2d(-1,1),
       Point2d(1,1),
       Point2d(1,-1)])
    
    poly_primitive = RenderPrimitive(polygon)
    renderer.Add("poly_prim_0", poly_primitive)
    renderer.Add("poly_prim_0", poly_primitive)
    renderer.Add("poly_prim_1", poly_primitive)
    
    # asserts
    rp = renderer.primitives
    self.assertTrue(len(rp) == 2)
    self.assertTrue(
      (polygon.ToArray() == rp["poly_prim_0"][0].object.ToArray()).all())
    self.assertTrue(
      (polygon.ToArray() == rp["poly_prim_0"][1].object.ToArray()).all())
    self.assertTrue(
      (polygon.ToArray() == rp["poly_prim_1"][0].object.ToArray()).all())
    


if __name__ == '__main__':
  unittest.main()