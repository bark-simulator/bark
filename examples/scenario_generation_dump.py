# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT




from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.commons.parameters import ParameterServer
import time
import os


param_server = ParameterServer()

scenario_generation = UniformVehicleDistribution(num_scenarios=3, random_seed=0, params=param_server)

scenario_generation.dump_scenario_list("test.bark_scenarios")