# Copyright (c) 2019 fortiss GmbH
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import matplotlib as mpl
if os.environ.get('DISPLAY', '') == '':
    print('no display found. Using non-interactive Agg backend')
    mpl.use('Agg')
from bark.geometry import *
from bark.geometry.standard_shapes import *
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
        l.addPoint(p)
        l.addPoint(p)
        l.addPoint(p)
        self.assertFalse(l.valid())
        print(l)

        p = Point2d(0.0, 1.0)
        l.addPoint([3, 4])
        l.addPoint([1, 4])
        self.assertTrue(l.valid())
        arr = l.toArray()
        print(arr)


        p = Polygon2d()
        p.addPoint([0, 0])
        p.addPoint([1, 0])
        p.addPoint([1, 1])
        # to few points
        self.assertFalse(p.valid())
        p.addPoint([0, 0])
        # wrong orientation
        self.assertFalse(p.valid())

        p = Polygon2d()
        p.addPoint([0, 0])
        p.addPoint([0, 1])
        p.addPoint([1, 1])
        p.addPoint([0, 0])
        # right orientation
        self.assertTrue(p.valid())

    def test_bounding_box(self):
        p = Polygon2d()
        p.addPoint([0, 0])
        p.addPoint([0, 1])
        p.addPoint([1, 1])
        p.addPoint([0, 0])
        # right orientation
        self.assertTrue(p.valid())

        bb = p.bounding_box
        self.assertTrue(np.array_equal(np.array([0,0]),np.array([bb[0].x(), bb[0].y()])))
        self.assertTrue(np.array_equal(np.array([1,1]),np.array([bb[1].x(), bb[1].y()])))

        p = Polygon2d()
        p.addPoint([2, 3])
        p.addPoint([2, 4])
        p.addPoint([3, 4])
        p.addPoint([2, 3])
        # right orientation
        self.assertTrue(p.valid())

        bb2 = p.bounding_box
        self.assertTrue(np.array_equal(np.array([2,3]),np.array([bb2[0].x(), bb2[0].y()])))
        self.assertTrue(np.array_equal(np.array([3,4]),np.array([bb2[1].x(), bb2[1].y()])))

        bb_merged = merge_bounding_boxes(bb, bb2)

        self.assertTrue(np.array_equal(np.array([0,0]),np.array([bb_merged[0].x(), bb_merged[0].y()])))
        self.assertTrue(np.array_equal(np.array([3,4]),np.array([bb_merged[1].x(), bb_merged[1].y()])))

    def test_distance(self):

        p1 = Point2d(1, 10)
        p2 = Point2d(1, 5)

        self.assertEqual(distance(p1, p2), 5)

        l = Line2d()
        l.addPoint(p1)
        l.addPoint(p2)

        p3 = Point2d(4, 15)
        p4 = Point2d(4, -5)

        l2 = Line2d()
        l2.addPoint(p3)
        l2.addPoint(p4)

        self.assertEqual(distance(l2, p3), 0)
        self.assertEqual(distance(l, l2), 3)

        p = Polygon2d()
        p.addPoint([0, 0])
        p.addPoint([0, 1])
        p.addPoint([1, 1])
        p.addPoint([0, 0])

        p2 = Polygon2d()
        p2.addPoint([5, 0])
        p2.addPoint([5, 1])
        p2.addPoint([6, 1])
        p2.addPoint([5, 0])

        self.assertEqual(distance(p, p2), 4)
        self.assertEqual(distance(p, l2), 3)
        self.assertEqual(distance(p2, Point2d(10, 1)), 4)

    # TODO: need assertion in this test
    def test_transformation(self):
        #viewer = MPViewer()

        p = Polygon2d()
        p.addPoint([0, 0])
        p.addPoint([0, 2])
        p.addPoint([4, 2])
        p.addPoint([4, 0])
        p.addPoint([0, 0])
        self.assertTrue(p.valid())

        # rotate around pi
        # viewer.drawPolygon2d(p.rotate(math.pi/2),'r',0.2)
        p.rotate(math.pi / 2)

        # rotate around top left corner (second point)
        p2 = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        for x in np.linspace(0, 2 * math.pi, 7):
            p2.rotate(x)
            #viewer.drawPolygon2d(p2.rotate(x), 'b', 0.2)

        # translate into the nirvana based on previous rotations and translations
        #viewer.drawPolygon2d(p2.translate(Point2d(10,0)), 'g', 0.2)
        #viewer.drawPolygon2d(p2.transform([5,5,0.4]), 'k', 0.2)
        #viewer.show()

    def test_const_poly_from_array(self):
        arr = np.array([[0, 0], [0, 2], [4, 2], [4, 0], [0, 0]])
        poly = Polygon2d([1, 3, 1], arr)
        self.assertTrue(poly.valid())

    def test_distances_to_center(self):

        shape = CarLimousine()

        self.assertAlmostEqual(shape.front_dist, 3.849999, places=4)
        self.assertAlmostEqual(shape.rear_dist, 1.118999, places=4)
        self.assertAlmostEqual(shape.left_dist, 0.955999, places=4)
        self.assertAlmostEqual(shape.right_dist, 0.955999, places=4)



if __name__ == '__main__':
    unittest.main()
