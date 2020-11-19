# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np

from scipy.special import fresnel
from bark.core.world.opendrive import PlanView, XodrLaneOffset, XodrLaneWidth,\
  XodrLane, XodrRoad, FresnelCos, FresnelSin
from bark.core.geometry import Point2d


class EnvironmentTests(unittest.TestCase):
  def test_line(self):
    pv = PlanView()

    # Line
    pv.AddLine(Point2d(0, 0), 1.57079632679, 10, 10)
    line = pv.GetReferenceLine().ToArray()

    # Spiral
    p = Point2d(line[-1][0], line[-1][1])
    pv.AddSpiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
    line = pv.GetReferenceLine().ToArray()

    offset = XodrLaneOffset(1.5, 0, 0, 0)
    lane_width = XodrLaneWidth(0.0, 59.9, offset)

    lane = XodrLane.CreateLaneFromLaneWidth(-1,
                                            pv.GetReferenceLine(), lane_width, 0.5)

    print(lane)
    lane = XodrLane.CreateLaneFromLaneWidth(
        1, pv.GetReferenceLine(), lane_width, 0.5)
    print(lane)

  def test_road(self):
    newXodrRoad = XodrRoad()
    newXodrRoad.id = 1
    newXodrRoad.name = "Autobahn A9"

    newPlanView = PlanView()
    newPlanView.AddLine(Point2d(0, 0), 1.57079632679, 10, 10)

    newXodrRoad.plan_view = newPlanView

    line = newXodrRoad.plan_view.GetReferenceLine().ToArray()

    # Spiral
    p = Point2d(line[-1][0], line[-1][1])
    newXodrRoad.plan_view.AddSpiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
    line = newXodrRoad.plan_view.GetReferenceLine().ToArray()

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
      x_odr.append(FresnelCos(t_i))
      y_odr.append(FresnelSin(t_i))

    x_odr_np = np.asarray(x_odr)
    y_odr_np = np.asarray(y_odr)

    assert (np.allclose(x, x_odr_np) == 1)
    assert (np.allclose(y, y_odr_np) == 1)


if __name__ == '__main__':
  unittest.main()
