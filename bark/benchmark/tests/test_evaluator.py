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

class ExperienceCollector(BaseEvaluator):
  def __init__(self, agent_id):
    super(ExperienceCollector, self).__init__()
    self._agent_id = agent_id

  def Evaluate(self, observed_world):
    if isinstance(observed_world, ObservedWorld):
      return 0.2
    else:
      return 0.3
