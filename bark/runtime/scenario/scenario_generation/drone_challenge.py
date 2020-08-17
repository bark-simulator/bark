# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.runtime.scenario import Scenario
from bark.runtime.scenario.scenario_generation import ScenarioGeneration
from bark.runtime.commons import ModelJsonConversion
from bark.core.world.agent import *
from bark.core.models.behavior import *
from bark.core.world import *
from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionSequential
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.runtime.commons import ParameterServer
from bark.runtime.commons import XodrParser

import numpy as np
import math


class DroneChallengeScenarioGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(DroneChallengeScenarioGeneration, self).__init__(params,
                                                          num_scenarios)
    self.initialize_params(params)

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["DeterministicScenarioGeneration"]
    
    self.goal_frame_center = self._local_params["goal_frame"]["center_pose", "Center pose of the goal frames", [0,0,0]]
    self.goal_frame_points = self._local_params["goal_frame"]["polygon_points",
                                                 "Points of the goal frame polygon",
                                                 [[-0.2,1],[0.2, 1],[0.2,-1],[-0.2,-1],[-0.2,1]]]
    self.goal_frame_poses = self._local_params["goal_poses", "A list with x,y, theta specyfing the sequence of the goal frames",
                    [[5,10, 3.14/2+0.01], [10,15, 3.14/4],[20,10, 3.14/2] ,[15,2, 3.14/2-0.7]]]

    json_converter = ModelJsonConversion()
    _agent_params = json_converter.agent_to_json(self.default_drone_model())
    if not isinstance(_agent_params, dict):
        _agent_params = _agent_params.ConvertToDict()

    self.drone_params = self._local_params["drones", "list of dictionaries with drone definitions", [{"drone_model": _agent_params}]]
    self.ego_agent_id = self._local_params["ego_agent_id","ID of the ego-agent", 0]

  def create_scenarios(self, params, num_scenarios):
    """ 
        see baseclass
    """
    scenario_list = []
    for scenario_idx in range(0, num_scenarios):
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    return scenario_list


  def _build_sequential_goal_definition(self):
    goal_list = []
    for goal_pose in self.goal_frame_poses:
        goal_polygon = Polygon2d(self.goal_frame_center,
                               np.array(self.goal_frame_points))
        goal_polygon = goal_polygon.Transform(goal_pose)
        goal_definition = GoalDefinitionPolygon(goal_polygon)
        goal_list.append(goal_definition)
    
    return GoalDefinitionSequential(goal_list)

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=None,
                        json_params=self._params.ConvertToDict())
    scenario._agent_list = []
    for agent_json in self.drone_params:
      agent_json["drone_model"]["map_interface"] = None
      agent_json["drone_model"]["goal_definition"] = self._build_sequential_goal_definition()
      json_converter = ModelJsonConversion()
      agent = json_converter.agent_from_json(agent_json["drone_model"],
                                                   param_server=self._local_params)
      agent.SetAgentId(agent_json["drone_model"]["id"])
      scenario._agent_list.append(agent)

    scenario._eval_agent_ids = [self._local_params["EgoAgentId",
                                "ID of the ego-agent",
                                0]]
    return scenario

  def default_drone_model(self):
    param_server = ParameterServer()
    dynamic_model = SingleTrackModel(param_server)
    behavior_model = BehaviorConstantAcceleration(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    drone_2d_shape = Polygon2d([0,0,0], np.array([
        [0.3,0.1],[0.3,-0.1],[0.1,-0.3],[-0.1,-0.3],[-0.3,-0.1],[-0.3,0.1],[-0.1,0.3],[0.1,0.3],[0.3,0.1]
    ]))
    init_state = np.array([0, 0, 0, 0, 0])

    drone_default = Agent(init_state,
                          behavior_model,
                          dynamic_model,
                          execution_model,
                          drone_2d_shape,
                          param_server)

    return drone_default