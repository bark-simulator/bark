# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import numpy as np

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

class BehaviorSpace:
  def __init__(self, params):
    self._params = params.AddChild("BehaviorSpace")
    self._behavior_space_definition = self._params.AddChild("Definition")
    self._sampling_parameters = self._params.AddChild("Sampling")
    self._random_seed = self._sampling_parameters["RandomSeed", "Seed for parameter sampling", 1000]
    self._config_behavior_space()
    self.random_state = np.random.RandomState(self._random_seed)

  def sample_behavior_parameters(self, random_state = None):
    if random_state:
      self.random_state = random_state
    return self._sample_params_from_param_ranges(self._behavior_space_range_params, \
                self._sampling_parameters), self.model_type


  def create_hypothesis_set(self):
    hypothesis_parameters = self._params.AddChild("Hypothesis")
    num_hypothesis = ["NumHypothesis", "Number of hypthesis", 20]
    pass

  def _config_behavior_space(self):
    self.model_type = self._behavior_space_definition["ModelType", "Model type over which behavior space is defined", \
        "BehaviorIDMStochasticHeadway"]
    self._behavior_space_range_params = self._behavior_space_definition.AddChild("SpaceBoundaries")
    model_params = ParameterServer()
    _, _ = self._model_from_model_type(self.model_type, model_params)

    def replace_with_ranges(model_params, space_boundary_params):
        for key, value in model_params.store.items():
          if "Distribution" in key:
              space_boundary_params[key] = [0, 1] # default range
              continue
          elif isinstance(value, ParameterServer):
            replace_with_ranges(value, space_boundary_params[key])
          else:
            space_boundary_params[key] = [value]  #

    replace_with_ranges(model_params, self._behavior_space_range_params)

  def _model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def _sample_params_from_param_ranges(self, space_params, sampling_params):
    """
    searches through param server to find distribution keys,
    adds by default to all distribution types a range parameter
    """

    param_dict = ParameterServer(log_if_default=True)
    for key, value in space_params.store.items():
      child = sampling_params[key]
      if "Distribution" in key:
        distribution_type = sampling_params[key]["DistributionType", "Distribution type for sampling", "UniformDistribution"]
        parameter_range = value
        if "Uniform" in distribution_type:
          param_dict[key] = self._sample_uniform_dist_params(parameter_range, child)
        elif "Gauss" in distribution_type:
          param_dict[key] = self._sample_gauss_dist_params(parameter_range, child)
      elif isinstance(value, ParameterServer):
        param_dict[key] = self._sample_params_from_param_ranges(value, child)
      else:
        parameter_range = value
        param_dict[key] = self._sample_non_distribution_params(parameter_range, child)
      if len(child.store) == 0:
        del sampling_params[key]
    return param_dict

  def _sample_non_distribution_params(self, range, sampling_params):
    param_sampled = None
    if isinstance(range, list):
      if len(range) > 1:
        param_sampled = self.random_state.uniform(range[0], range[1])
      else:
        param_sampled = range[0]
    else:
      param_sampled = range
    return param_sampled

  def _sample_uniform_dist_params(self, range, sampling_params):
    uni_width = sampling_params["Width", "What minimum and maximum width should sampled distribution have", [0.1, 0.3]]

    lower_bound = self.random_state.uniform(range[0], range[1] - uni_width[0])
    upper_bound = self.random_state.uniform(lower_bound + uni_width[0],  lower_bound + uni_width[1])

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "UniformDistribution1D"
    sampled_params["LowerBound"] = lower_bound
    sampled_params["UpperBound"] = upper_bound
    sampled_params["RandomSeed"] = sampling_params["RandomSeed", "Seed for stochastic behavior", 1000]
    return sampled_params


  def _sample_gauss_dist_params(self, range, sampling_params):
    return {}