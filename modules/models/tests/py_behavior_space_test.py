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
    param_server.save("behavior_space_defaults_sampling.json")

    params_loaded = ParameterServer(filename="behavior_space_defaults_sampling.json")
    space = BehaviorSpace(params_loaded)
    sampled_parameters = space.sample_behavior_parameters()

  def test_default_config_hypothesis_creation(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    num_hypothesis_desired = param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochasticHeadway"]["HeadwayDistribution"]
    self.assertEqual(len(hypothesis_set), num_hypothesis_desired)

    default_range = [3.0, 4.0]
    for idx, hypothesis in enumerate(hypothesis_set):
      params = hypothesis.params
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::LowerBound", "", 0.0), \
                 default_range[0] + idx*1/num_hypothesis_desired, 5)
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::UpperBound", "", 0.0),\
                 default_range[0] + (idx+1)*1/num_hypothesis_desired, 5)

    param_server.save("behavior_space_defaults_hypothesis.json")
    params_loaded = ParameterServer(filename="behavior_space_defaults_hypothesis.json")
    space = BehaviorSpace(params_loaded)
    _,_ = space.create_hypothesis_set()

if __name__ == '__main__':
  unittest.main()