# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import numpy as np
import math

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderObserverModel
from bark.runtime.commons.parameters import ParameterServer
from bark.core.models.observer import ObserverModelNone, ObserverModelParametric


# observed worlds with no perturbations
class ObserverModelNoneReader(ConfigReaderObserverModel):
  # returns a observer model none for the world
  def create_from_config(self, config_param_object,  **kwargs):
    observer_model = ObserverModelNone(config_param_object)
    return observer_model, {}, config_param_object

# observed worlds with parametric perturbations
class ObserverModelParametricReader(ConfigReaderObserverModel):
  # returns a parametric observer model for the world
  def create_from_config(self, config_param_object, **kwargs):
    observer_model_parametric = ObserverModelParametric(config_param_object)
    return observer_model_parametric, {}, config_param_object
