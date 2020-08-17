# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import os
import numpy as np
import matplotlib as mpl
if os.environ.get('DISPLAY', '') == '':
  print('no display found. Using non-interactive Agg backend')
  mpl.use('Agg')
from bark.core.world.agent import Agent
from bark.core.models.behavior import BehaviorConstantAcceleration
from bark.core.models.dynamic import SingleTrackModel
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.geometry import Polygon2d, Point2d
from bark.core.geometry.standard_shapes import CarLimousine
from bark.runtime.commons.parameters import ParameterServer


class AgentTests(unittest.TestCase):
  def test_write_params_agent(self):
    params = ParameterServer()
    behavior = BehaviorConstantAcceleration(params)
    execution = ExecutionModelInterpolate(params)
    dynamic = SingleTrackModel(params)
    shape = Polygon2d([1.25, 1, 0], [
        Point2d(0, 0),
        Point2d(0, 2),
        Point2d(4, 2),
        Point2d(4, 0),
        Point2d(0, 0)
    ])
    init_state = np.zeros(4)
    agent = Agent(init_state, behavior, dynamic, execution, shape,
                  params.AddChild("agent"))
    params.Save("written_agents_param_test.json")

  def test_draw_agents(self):
    params = ParameterServer()
    behavior = BehaviorConstantAcceleration(params)
    execution = ExecutionModelInterpolate(params)
    dynamic = SingleTrackModel(params)
    shape = Polygon2d([1.25, 1, 0], [
        Point2d(0, 0),
        Point2d(0, 2),
        Point2d(4, 2),
        Point2d(4, 0),
        Point2d(0, 0)
    ])
    shape2 = CarLimousine()

    init_state = [0, 3, 2, 1]
    init_state2 = [0, 0, 5, 4]

    agent = Agent(init_state, behavior, dynamic, execution, shape,
                  params.AddChild("agent"))
    agent2 = Agent(init_state2, behavior, dynamic, execution, shape2,
                    params.AddChild("agent"))



if __name__ == '__main__':
  unittest.main()
