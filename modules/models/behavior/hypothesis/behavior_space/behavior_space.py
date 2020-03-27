# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import numpy as np

from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

class BehaviorSpace:
  def __init__(self, params):
    self._params = params.AddChild("BehaviorSpace")
    self._behavior_space_definition = self._params.AddChild("Definition")
    self._sampling_parameters = self._params.AddChild("Sampling")
    self._hypothesis_parameters = self._params.AddChild("Hypothesis")
    self._config_behavior_space()

  def sample_behavior_parameters(self):
    return self._sample_params_from_param_ranges()


  def create_hypothesis_set(self):
    num_hypothesis = self._hypothesis_parameters["NumHypothesis", "Number of hypthesis", 20]
    pass

  def _config_behavior_space(self):
    model_type = self._behavior_space_definition["ModelType", "Type of behavior model \
                used for all vehicles", "BehaviorIDMStochasticHeadway"]
    self._behavior_space_range_params = self._behavior_space_definition.AddChild("SpaceBoundaries")
    model_params = ParameterServer()
    _, _ = self._model_from_model_type(model_type, model_params)

    def replace_with_ranges(model_params, space_boundary_params):
        for key, value in model_params.store.items():
          if isinstance(value, ParameterServer):
            replace_with_ranges(value, space_boundary_params[key])
          else:
            if "Distribution" in key:
              space_boundary_params[key] = [0, 1] # default range
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

    param_dict = ParameterServer()
    for key, value in space_params.store.items():
      if "Distribution" in key:
        distribution_type = sampling_params[key]["DistributionType", "Distribution type for sampling", "UniformDistribution"]
        parameter_range = value
        if "Uniform" in distribution_type:
          param_dict[key] = self._sample_uniform_dist_params(parameter_range, value)
        elif "Gauss" in distribution_type:
          param_dict[key] = self._sample_gauss_dist_params(parameter_range, value)
      elif isinstance(value, ParameterServer):
        param_dict[key] = self._sample_params_from_param_ranges(value, sampling_params[key])
      else:
        param_dict[key] = value
    return param_dict

  def _sample_uniform_dist_params(self, range, sampling_params):
    uni_width = sampling_params["Width", "What minimum and maximum width should sampled distribution have", [0.1, 0.3]]

    lower_bound = self.random_state.uniform(range[0], range[1] - uni_width[0])
    upper_bound = self.random_state.uniform(lower_bound + uni_width[0],  lower_bound + uni_width[1])

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["LowerBound"] = lower_bound
    sampled_params["UpperBound"] = upper_bound
    sampled_params["RandomSeed"] = params_distribution["RandomSeed"]
    return sampled_params


  def _sample_gauss_dist_params(self, range, sampling_params):
    return {}