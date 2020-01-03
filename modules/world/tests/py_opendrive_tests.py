# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import numpy as np

from scipy.special import fresnel
from bark.world import *
from bark.world.opendrive import PlanView, LaneOffset, LaneWidth, Lane, Road, fresnel_cos, fresnel_sin
from bark.geometry import Point2d


class EnvironmentTests(unittest.TestCase):
    def test_line(self):
        pv = PlanView()

        # Line
        pv.add_line(Point2d(0, 0), 1.57079632679, 10)
        line = pv.get_reference_line().toArray()

        # Spiral
        p = Point2d(line[-1][0], line[-1][1])
        pv.add_spiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
        line = pv.get_reference_line().toArray()

        offset = LaneOffset(1.5, 0, 0, 0)
        lane_width = LaneWidth(0.0, 59.9, offset)

        lane = Lane.create_lane_from_lane_width(-1,
                                                pv.get_reference_line(), lane_width, 0.5)

        print(lane)
        lane = Lane.create_lane_from_lane_width(
            1, pv.get_reference_line(), lane_width, 0.5)
        print(lane)

    def test_road(self):
        newRoad = Road()
        newRoad.id = 1
        newRoad.name = "Autobahn A9"

        newPlanView = PlanView()
        newPlanView.add_line(Point2d(0, 0), 1.57079632679, 10)

        newRoad.plan_view = newPlanView

        line = newRoad.plan_view.get_reference_line().toArray()

        # Spiral
        p = Point2d(line[-1][0], line[-1][1])
        newRoad.plan_view.add_spiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
        line = newRoad.plan_view.get_reference_line().toArray()

    def test_spiral(self):
        '''
        spiral test compares outcome of wrapped odrSpiral implementation with 
        scipy fresnel calculation
        '''

        # spiral using scipy
        t = np.linspace(-7, 7, 250)
        y, x = fresnel(t)

        # spiral using odrSpiral
        x_odr = []
        y_odr = []
        for t_i in t:
            x_odr.append(fresnel_cos(t_i))
            y_odr.append(fresnel_sin(t_i))

        x_odr_np = np.asarray(x_odr)
        y_odr_np = np.asarray(y_odr)

        assert (np.allclose(x, x_odr_np) == 1)
        assert (np.allclose(y, y_odr_np) == 1)


if __name__ == '__main__':
    unittest.main()
