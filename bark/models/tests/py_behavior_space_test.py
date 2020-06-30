# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try:
  import debug_settings
except:
  pass


import unittest
import os
import numpy as np
from collections import defaultdict
from operator import itemgetter

from bark.runtime.commons.parameters import ParameterServer
from bark.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace
from bark.core.models.behavior import *

class PyBehaviorSpaceTests(unittest.TestCase):
  def test_default_config_sampling(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    sampled_parameters, model_type = space.sample_behavior_parameters()
    print(model_type)
    behavior = eval("{}(sampled_parameters)".format(model_type))
    print(sampled_parameters.ConvertToDict())
    param_server.Save("behavior_space_defaults_sampling.json")

    params_loaded = ParameterServer(filename="behavior_space_defaults_sampling.json")

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.23, 20.2]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["StdRange"] = [0.2, 0.4]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType"] = "NormalDistribution1D"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [4.5, 6.07]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Width"] = [0.1, 0.3]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType"] = "UniformDistribution1D"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [1.7]

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [20]
    space = BehaviorSpace(params_loaded) 

    num_sampled_parameters = 100
    for _ in range(0, num_sampled_parameters):
      sampled_parameters, model_type = space.sample_behavior_parameters()
      behavior = eval("{}(sampled_parameters)".format(model_type))

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["FixedValue", "", 1.0], [1.7]) 

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["FixedValue", "", 1.2], [20])
      
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType", "", ""], "NormalDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["Std", "", 0.2] <= 0.4)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["Std", "", 0.1] >= 0.2)

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType", "", ""], "UniformDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]>= 4.5)
      lb = sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 0.2] >= lb+0.1)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 0.1] <= lb+0.3)

  def test_default_config_hypothesis_creation(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    num_hypothesis_desired = param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochastic"]["HeadwayDistribution"]
    self.assertEqual(len(hypothesis_set), num_hypothesis_desired)

    default_range = [3.0, 4.0]
    for idx, hypothesis in enumerate(hypothesis_set):
      params = hypothesis.params
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0), \
                 default_range[0] + idx*1/num_hypothesis_desired, 5)
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0),\
                 default_range[0] + (idx+1)*1/num_hypothesis_desired, 5)

    param_server.Save("behavior_space_defaults_hypothesis.json")
    params_loaded = ParameterServer(filename="behavior_space_defaults_hypothesis.json")
    space = BehaviorSpace(params_loaded)
    _,_ = space.create_hypothesis_set()

  def test_cover_hypothesis_creation(self):
    param_server = ParameterServer(log_if_default=True)
    behavior_space_range1 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [5.34, 10.0]
    behavior_space_range2 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.3434, 10.0]
    behavior_space_fixed_val = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [3.4545]
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_cover_hypothesis()
    num_hypothesis_desired = param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochastic"]["HeadwayDistribution"]
    self.assertEqual(len(hypothesis_set), 1)

    params = hypothesis_set[0].params
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0), \
                behavior_space_range1[0], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0),\
                 behavior_space_range1[1], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::SpacingDistribution::LowerBound", "", 0.0), \
                behavior_space_range2[0], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::SpacingDistribution::UpperBound", "", 0.0),\
                 behavior_space_range2[1], 5)
    self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::DesiredVelDistribution::FixedValue", "", [0.0])[0],\
                 behavior_space_fixed_val[0], 5)

  def test_multiple_hypothesis_sets_creation(self):
    param_server = ParameterServer()
    behavior_space_range1 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [5.3434, 10.14]
    behavior_space_range2 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [1.0]
    behavior_space_range3 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [15.3]
    behavior_space_range4 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.0, 2.0]
    behavior_space_range5 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [3, 4.5]
    behavior_space_range6 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"] = [ 0.99 ]
    space = BehaviorSpace(param_server)
    desired_splits = [2, 8]
    hypothesis_set_collection = space.create_multiple_hypothesis_sets(splits=[2, 8])

    self.assertEqual(len(hypothesis_set_collection), len(desired_splits))

    bounds_collected_split = {}
    for split, (hypothesis_set, params_sets) in hypothesis_set_collection.items():
      num_hypothesis_desired = split
      self.assertEqual(len(hypothesis_set), split**3)
      bounds_collected_split[split] = defaultdict(list)
      bounds_collected = bounds_collected_split[split]
      for idx, hypothesis in enumerate(hypothesis_set):
        params = hypothesis.params
        lower = params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0)
        bounds_collected[1].append([lower, upper])

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue", "", [4345.0])[0], \
                  behavior_space_range2[0], 5)

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::MaxAccDistribution::FixedValue", "", [2334.0])[0], \
                  behavior_space_range3[0], 5)

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue", "", [2334.0])[0], \
                  behavior_space_range6[0], 5)

        lower = params.getReal("BehaviorIDMStochastic::SpacingDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::SpacingDistribution::UpperBound", "", 0.0)
        bounds_collected[4].append([lower, upper])

        lower = params.getReal("BehaviorIDMStochastic::DesiredVelDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::DesiredVelDistribution::UpperBound", "", 0.0)
        bounds_collected[5].append([lower, upper])

    desired_ranges = {1: behavior_space_range1, 4 : behavior_space_range4, 5 : behavior_space_range5}
    for split in desired_splits: 
      bounds_collected = bounds_collected_split[split]
      num_hypothesis_desired = split
      for behavior_range, bounds in bounds_collected.items():
          tmp = set(tuple(bound) for bound in bounds)
          splits = sorted(list(tmp), key=itemgetter(0))
          for idx, split in enumerate(splits):
            desired_range = desired_ranges[behavior_range]
            self.assertAlmostEquals(split[0], \
                      desired_range[0] + idx*(desired_range[1]-desired_range[0])/num_hypothesis_desired, 2)
            self.assertAlmostEquals(split[1],\
                      desired_range[0] + (idx+1)*(desired_range[1]-desired_range[0])/num_hypothesis_desired, 2)


if __name__ == '__main__':
  unittest.main()