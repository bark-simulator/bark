# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.runtime.commons.parameters import ParameterServer

from bark.core.world.evaluation import *
from bark.core.world import *

class TestPythonEvaluator(BaseEvaluator):
  def __init__(self, params, agent_id):
    super(TestPythonEvaluator, self).__init__()
    self._agent_id = agent_id

  def SetAgentId(self, agent_id):
    self._agent_id = agent_id

  def Evaluate(self, observed_world):
    if isinstance(observed_world, ObservedWorld):
      return (0.2, 0.3, 0.4, [0.1, 0.2])
    else:
      return (0.2, 0.5, 0.4, [0.1, 0.2])


class TestPythonEvaluatorSerializable(BaseEvaluator):
  def __init__(self, some_params_to_set = {"test1" : 2456}):
    super(TestPythonEvaluatorSerializable, self).__init__()
    self._some_params_to_set = some_params_to_set

  def SetAgentId(self, agent_id):
    self._agent_id = agent_id

  def Evaluate(self, observed_world):
    if isinstance(observed_world, ObservedWorld):
      return (0.2, 0.3, 0.4, [0.1, 0.2])
    else:
      obs_world = observed_world.Observe([self._agent_id])[0]
      return (0.2, 0.5, 0.4, [0.1, 0.2])

  def __setstate__(self, d):
    pass

  def __getstate__(self):
    return {}