# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import math
from bark.core.geometry.standard_shapes import *
from bark.core.geometry import *
import unittest
import os
import matplotlib as mpl
if os.environ.get('DISPLAY', '') == '':
    print('no display found. Using non-interactive Agg backend')
    mpl.use('Agg')
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
import math
import numpy as np

#from bark.visualization import MPViewer


class GeometryTests(unittest.TestCase):
    def test_init_objects(self):
        p = Point2d(0.0, 1.0)
        print(p)
        print(p.x)
        print(p.y)

        l = Line2d()
        l.AddPoint(p)
        l.AddPoint(p)
        l.AddPoint(p)
        self.assertFalse(l.Valid())
        print(l)

        p = Point2d(0.0, 1.0)
        l.AddPoint([3, 4])
        l.AddPoint([1, 4])
        self.assertTrue(l.Valid())
        arr = l.ToArray()
        print(arr)

        p = Polygon2d()
        p.AddPoint([0, 0])
        p.AddPoint([1, 0])
        p.AddPoint([1, 1])
        # to few points
        self.assertFalse(p.Valid())
        p.AddPoint([0, 0])
        # wrong orientation
        self.assertFalse(p.Valid())

        p = Polygon2d()
        p.AddPoint([0, 0])
        p.AddPoint([0, 1])
        p.AddPoint([1, 1])
        p.AddPoint([0, 0])
        # right orientation
        self.assertTrue(p.Valid())

    def test_bounding_box(self):
        p = Polygon2d()
        p.AddPoint([0, 0])
        p.AddPoint([0, 1])
        p.AddPoint([1, 1])
        p.AddPoint([0, 0])
        # right orientation
        self.assertTrue(p.Valid())

        bb = p.bounding_box
        self.assertTrue(np.array_equal(
            np.array([0, 0]), np.array([bb[0].x(), bb[0].y()])))
        self.assertTrue(np.array_equal(
            np.array([1, 1]), np.array([bb[1].x(), bb[1].y()])))

        p = Polygon2d()
        p.AddPoint([2, 3])
        p.AddPoint([2, 4])
        p.AddPoint([3, 4])
        p.AddPoint([2, 3])
        # right orientation
        self.assertTrue(p.Valid())

        bb2 = p.bounding_box
        self.assertTrue(np.array_equal(
            np.array([2, 3]), np.array([bb2[0].x(), bb2[0].y()])))
        self.assertTrue(np.array_equal(
            np.array([3, 4]), np.array([bb2[1].x(), bb2[1].y()])))

        bb_merged = MergeBoundingBoxes(bb, bb2)

        self.assertTrue(np.array_equal(np.array([0, 0]), np.array(
            [bb_merged[0].x(), bb_merged[0].y()])))
        self.assertTrue(np.array_equal(np.array([3, 4]), np.array(
            [bb_merged[1].x(), bb_merged[1].y()])))

    def test_Distance(self):

        p1 = Point2d(1, 10)
        p2 = Point2d(1, 5)

        self.assertEqual(Distance(p1, p2), 5)

        l = Line2d()
        l.AddPoint(p1)
        l.AddPoint(p2)

        p3 = Point2d(4, 15)
        p4 = Point2d(4, -5)

        l2 = Line2d()
        l2.AddPoint(p3)
        l2.AddPoint(p4)

        self.assertEqual(Distance(l2, p3), 0)
        self.assertEqual(Distance(l, l2), 3)

        p = Polygon2d()
        p.AddPoint([0, 0])
        p.AddPoint([0, 1])
        p.AddPoint([1, 1])
        p.AddPoint([0, 0])

        p2 = Polygon2d()
        p2.AddPoint([5, 0])
        p2.AddPoint([5, 1])
        p2.AddPoint([6, 1])
        p2.AddPoint([5, 0])

        self.assertEqual(Distance(p, p2), 4)
        self.assertEqual(Distance(p, l2), 3)
        self.assertEqual(Distance(p2, Point2d(10, 1)), 4)

    # TODO: need assertion in this test
    def test_transformation(self):
        #viewer = MPViewer()

        p = Polygon2d()
        p.AddPoint([0, 0])
        p.AddPoint([0, 2])
        p.AddPoint([4, 2])
        p.AddPoint([4, 0])
        p.AddPoint([0, 0])
        self.assertTrue(p.Valid())

        # rotate around pi
        # viewer.drawPolygon2d(p.Rotate(math.pi/2),'r',0.2)
        p.Rotate(math.pi / 2)

        # rotate around top left corner (second point)
        p2 = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        for x in np.linspace(0, 2 * math.pi, 7):
            p2.Rotate(x)
            #viewer.drawPolygon2d(p2.Rotate(x), 'b', 0.2)

        # translate into the nirvana based on previous rotations and translations
        #viewer.drawPolygon2d(p2.Translate(Point2d(10,0)), 'g', 0.2)
        #viewer.drawPolygon2d(p2.Transform([5,5,0.4]), 'k', 0.2)
        # viewer.show()

    def test_const_poly_from_array(self):
        arr = np.array([[0, 0], [0, 2], [4, 2], [4, 0], [0, 0]])
        poly = Polygon2d([1, 3, 1], arr)
        self.assertTrue(poly.Valid())

    def test_concatenate_polygons(self):

        poly1 = Polygon2d()
        poly1.AddPoint([0, 0])
        poly1.AddPoint([0, 2])
        poly1.AddPoint([4, 2])
        poly1.AddPoint([4, 0])
        poly1.AddPoint([0, 0])
        self.assertTrue(poly1.Valid())
        a1 = poly1.CalculateArea()

        poly2 = Polygon2d()
        poly2.AddPoint([4, 0])
        poly2.AddPoint([4, 2])
        poly2.AddPoint([8, 2])
        poly2.AddPoint([8, 0])
        poly2.AddPoint([4, 0])
        self.assertTrue(poly2.Valid())
        a2 = poly2.CalculateArea()

        poly1.ConcatenatePolygons(poly2)
        self.assertTrue(poly1.Valid())

        a3 = poly1.CalculateArea()

        self.assertTrue(a3 == a1+a2)

    def test_robust_concatenate_polygons(self):

        poly1 = Polygon2d()
        poly1.AddPoint([0, 0])
        poly1.AddPoint([0, 2])
        poly1.AddPoint([4, 2])
        poly1.AddPoint([4, 0])
        poly1.AddPoint([0, 0])
        self.assertTrue(poly1.Valid())
        a1 = poly1.CalculateArea()

        eps = 0.05
        poly2 = Polygon2d()
        poly2.AddPoint([4+eps, 0])
        poly2.AddPoint([4+eps, 2])
        poly2.AddPoint([8+eps, 2])
        poly2.AddPoint([8+eps, 0])
        poly2.AddPoint([4+eps, 0])
        self.assertTrue(poly2.Valid())
        a2 = poly2.CalculateArea()

        poly1b = poly1.BufferPolygon(0.1)
        poly1b.ConcatenatePolygons(poly2)
        poly3 = poly1b.BufferPolygon(-0.1)
        self.assertTrue(poly3.Valid())

        a3 = poly3.CalculateArea()
        print(poly3)

    def test_distances_to_center(self):

        shape = CarLimousine()

        self.assertAlmostEqual(shape.front_dist, 3.849999, places=4)
        self.assertAlmostEqual(shape.rear_dist, 1.118999, places=4)
        self.assertAlmostEqual(shape.left_dist, 0.955999, places=4)
        self.assertAlmostEqual(shape.right_dist, 0.955999, places=4)

    def test_bounding_box_polygon(self):

        shape = CarLimousine()

        bbox_poly = CalculateBoundingBoxPolygon(shape)
        self.assertTrue(Within(shape, bbox_poly))

if __name__ == '__main__':
    unittest.main()
