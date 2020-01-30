# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT




from config_readers_interfaces import ConfigReaderBehaviorModels

  import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser


class UniformDistribution(ConfigReaderBehaviorModels):
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    behavior_model = BehaviorConstantVelocity(param_server)

    behavior_models = []
    for _ in agent_states:
