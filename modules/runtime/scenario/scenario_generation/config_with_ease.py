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


class LaneCorridorConfig:
  def __init__(self,
               road_ids=None,
               lane_corridor_id=0,
               params=None):
    self._road_corridor = None
    self._road_ids = road_ids
    self._lane_corridor_id = lane_corridor_id
    self._params = params
    self._current_s = None
  
  def state(self, world):
    pose = self.position(world)
    if pose is None:
      return None
    velocity = self.velocity()
    return np.array([0, pose[0], pose[1], pose[2], velocity])

  def position(self, world, min_distance=5., min_s=0., max_s=1.):
    if self._road_corridor == None:
      world.GenerateRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
    lane_corr = world.map.GetLaneCorridor(self._lane_corridor_id)
    centerline = lane_corr.center_line
    if self._current_s == None:
      self._current_s = min_s
    xy_point =  GetPointAtS(centerline, self._current_s)
    angle = GetTangentAngleAtS(centerline, self._current_s)
    if self._current_s > max_s:
      return None
    self._current_s += 0.1
    return Pose(xy_point.x(), xy_point.y(), angle)

  @property
  def velocity(self, min_vel=10., max_vel=15.):
    return np.random.uniform(low=min_vel, high=max_vel)

  @property
  def behavior_model(self, agent_controlled=False):
    """Returns behavior model
    """
    return BehaviorConstantVelocity(self._params)

  def execution_model(self, agent_controlled=False):
    """Returns exec. model
    """
    return ExecutionModelInterpolate(self._params)

  def dynamic_model(self, agent_controlled=False):
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

  def controlled(self, world):
    """Returns bool
    """
    return False

  def goal(self, world, agent_controlled=False):
    """Returns goal def.
    """
    # TODO(@hart): set different goal for controlled agent
    lane_corr = world.map.GetLaneCorridor(self._lane_corridor_id)
    return GoalDefinitionStateLimitsFrenet(lane_corr.center_line,
                                           (0.2, 0.2),
                                           (0.1, 0.1),
                                           (10., 15.))

  def reset(self):
    self._current_s = None


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
    self._lane_corridor_configs = lane_corridor_configs or []

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["ConfigWithEase"]
    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "modules/runtime/tests/data/Crossing8Course.xodr"]

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
    map_interface = world.map

    # fill agent list of the BARK world and set agents that are controlled
    scenario._agent_list = []
    scenario._eval_agent_ids = []
    for lc_config in self._lane_corridor_configs:
      agent_state = lc_config.state(world)
      if agent_state is not None:
        agent_controlled = lc_config.controlled(world)
        agent_behavior = lc_config.behavior_model(agent_controlled, agent_controlled)
        agent_dyn = lc_config.dynamic_model(agent_controlled, agent_controlled)
        agent_exec = lc_config.execution_model(agent_controlled, agent_controlled)
        agent_polygon = Polygon2d([0, 0, 0], lc_config.shape)
        agent_params = self._params.addChild("agent")
        agent_goal = lc_config.goal(world, agent_controlled)

        new_agent = Agent(
          agent_state, 
          agent_behavior, 
          agent_dyn,
          agent_exec, 
          agent_polygon,
          agent_params,
          agent_goal,
          map_interface)
        
        if agent_controlled:
          scenario._eval_agent_ids.append(new_agent.id)
        scenario._agent_list.append(new_agent)
    return scenario