# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion\
  import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits, GoalDefinitionSequential
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np
import math

# bark_agent = Agent(
#   np.array(agent_json["state"]), 
#   self.convert_model(agent_json["behavior_model"], param_server), 
#   self.convert_model(agent_json["dynamic_model"], param_server),
#   self.convert_model(agent_json["execution_model"], param_server), 
#   Polygon2d(agent_json["shape"]["center_pose"],
#   np.array(agent_json["shape"]["polygon_points"])),
#   param_server.addChild("agent"),
#   agent_json["goal_definition"],
#   agent_json["map_interface"])

class LaneCorridorConfig:
  def __init__(self,
               road_ids=None,
               lane_corridor_id=0,
               params=None):
    self._road_corridor = None
    self._road_ids = road_ids
    self._lane_corridor_id = lane_corridor_id
    self._params = params
  
  @property
  def position(self, world, min_distance=5., min_s=0., max_s=1.):
    if self._road_corridor == None:
      world.GenerateRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
    lane_corr = world.map.GetLaneCorridor(self._lane_corridor_id)
    centerline = lane_corr.centerline

    return Pose(0, 0., 0.)

  @property
  def velocity(self, centerline, min_vel=10., max_vel=15.):
    return np.random.uniform(low=min_vel, high=max_vel)

  @property
  def behavior_model(self):
    """Returns behavior model
    """
    return BehaviorConstantVelocity(self._params)

  @property
  def execution_model(self):
    """Returns exec. model
    """
    return ExecutionModelInterpolate(self._params)

  @property
  def dynamic_model(self):
    """Returns dyn. model
    """
    return SingleTrackModel(self._params)

  @property
  def shape(self):
    """Returns shape
    """
    return Polygon2d([Point2d(-2, -1),
                      Point2d(2, -1),
                      Point2d(2, 1),
                      Point2d(-2, 1),
                      Point2d(-2, -1)])

  @property
  def goal(self, world):
    """Returns goal def.
    """
    lane_corr = world.map.GetLaneCorridor(self._lane_corridor_id)


    pass

  @property
  def controlled(self, world):
    """Returns bool
    """
    return False


class ConfigWithEase(ScenarioGeneration):
  """Configure your scenarios with ease
     &copy; Patrick Hart
  """
  def __init__(self,
               num_scenarios,
               params=None,
               random_seed=None,
               lane_corridor_configs=None):
    super(ConfigWithEase, self).__init__(params, num_scenarios)
    self.initialize_params(params)
    self._lane_corridor_configs = lane_corridor_configs

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["ConfigWithEase"]
    
    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "modules/runtime/tests/data/Crossing8Course.xodr"]
    self._json_converter = ModelJsonConversion()

  def create_scenarios(self, params, num_scenarios):
    """ 
        see baseclass
    """
    scenario_list = []
    for scenario_idx in range(0, num_scenarios):
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    return scenario_list

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()
    agent_json["map_interface"] = world.map

    # where, how, where to
    # world.map.GenerateRoadCorridor(start_road_id, end_road_id)

    # FOR EACH LANE_CORR
    # 1. place_agents(centerline)
    # 2. assign_models(agents)
    # 3. set_controlled_agent(agents)
    for lc_config in self._lane_corridor_configs:
      vehicle_positions = lc_config.vehicle_positions
      vehicle_models = lc_config.vehicle_models
      controlled_agent_ids = lc_config.controlled_agents

      
    agent_list = []
    scenario._agent_list = []
    for agent_json_ in self._local_params["Agents"]:
      agent_json = agent_json_["VehicleModel"].copy()
      goal_polygon = Polygon2d([0, 0, 0],
                               np.array(agent_json["goal"]["polygon_points"]))
      goal_polygon = goal_polygon.Translate(Point2d(agent_json["goal"]["center_pose"][0],
                                                    agent_json["goal"]["center_pose"][1]))

      sequential_goals = []
      goal = GoalDefinitionPolygon(goal_polygon)
      if "goal_type" in agent_json["goal"]:
        goal_type = agent_json["goal"]["goal_type"]
        if goal_type == "GoalDefinitionStateLimits":
          goal = GoalDefinitionStateLimits(goal_polygon, (1.49, 1.65))

      # state_limit_goal = GoalDefinitionStateLimits(goal_polygon, (1.49, 1.65))
      for _ in range(self._local_params["goal"]["num_reached", "num", 5]):
        sequential_goals.append(goal)
      sequential_goal = GoalDefinitionSequential(sequential_goals)
      agent_json["goal_definition"] = sequential_goal

      agent_state = np.array(agent_json["state"])
      if len(np.shape(agent_state)) > 1:
        agent_state = np.random.uniform(low=agent_state[:, 0],
                                        high=agent_state[:, 1])
      agent_json["state"] = agent_state.tolist()
      agent = self._json_converter.agent_from_json(agent_json,
                                                   param_server=self._params)
      agent.SetAgentId(agent_json["id"])
      scenario._agent_list.append(agent)
    
    # TODO(@hart): this could be mult. agents
    scenario._eval_agent_ids = self._local_params["controlled_ids",
                                "IDs of agents to be controlled. ",
                                [0]]
    return scenario