# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
import numpy as np
from modules.runtime.commons.parameters import ParameterServer
from modules.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace

from bark.models.behavior import *

class PyBehaviorSpaceTests(unittest.TestCase):
  def test_default_config_sampling(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    sampled_parameters, model_type = space.sample_behavior_parameters()
    print(model_type)
    behavior = eval("{}(sampled_parameters)".format(model_type))
    print(sampled_parameters.convert_to_dict())
    param_server.save("behavior_space_defaults.json")

    params_loaded = ParameterServer(filename="behavior_space_defaults.json")
    space = BehaviorSpace(params_loaded)
    sampled_parameters = space.sample_behavior_parameters()

  def test_default_config_hypothesis_creation(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    self.assertEqual(len(hypothesis_set), param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochasticHeadway"]["HeadwayDistribution"])




if __name__ == '__main__':
  unittest.main()