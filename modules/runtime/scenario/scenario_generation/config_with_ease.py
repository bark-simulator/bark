# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration
from modules.runtime.commons.model_json_conversion import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import *
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.world.opendrive import *

import numpy as np
import math


class LaneCorridorConfig:
  """This class enables the configuration of a single LaneCorridor
     It assigns all models for the agents, determines their positions and more.
     Additionally, it can be chosen which agents should be controlled,
     what goal and model they should have.
  """
  def __init__(self,
               params=None,
               **kwargs):
    self._road_corridor = None
    self._params = params
    self._current_s = None

    # set these params
    self._road_ids = kwargs.pop("road_ids", None)
    self._lane_corridor_id = kwargs.pop("lane_corridor_id", None)
    self._s_min = kwargs.pop("s_min", 0.) 
    self._s_max = kwargs.pop("s_max", 60.)
    self._ds_min = kwargs.pop("ds_min", 10.)
    self._ds_max = kwargs.pop("ds_max", 20.)
    self._min_vel = kwargs.pop("min_vel", 8.)
    self._max_vel = kwargs.pop("max_vel", 10.)
    self._source_pos = kwargs.pop("source_pos", None)
    self._sink_pos = kwargs.pop("sink_pos", None)
    self._behavior_model = \
      kwargs.pop("behavior_model", BehaviorIDMClassic(self._params))
    self._controlled_behavior_model = \
      kwargs.pop("controlled_behavior_model", None)
    self._controlled_ids = kwargs.pop("controlled_ids", None)

  def InferRoadIdsAndLaneCorr(self, world):
    goal_polygon = Polygon2d([0, 0, 0],
                             [Point2d(-1,0),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,0)])
    start_point = Point2d(self._source_pos[0], self._source_pos[1])
    end_point = Point2d(self._sink_pos[0], self._sink_pos[1])
    goal_polygon = goal_polygon.Translate(end_point)
    self._road_corridor = world.map.GenerateRoadCorridor(
      start_point, goal_polygon)
    self._road_ids = self._road_corridor.road_ids
    print("RoadIds: ", self._road_ids)
    self._lane_corridor = self._road_corridor.GetCurrentLaneCorridor(
      start_point)
    
  def state(self, world):
    """Returns a state of the agent
    
    Arguments:
        world {BARK.world}
    
    Returns:
        np.array -- time, x, y, theta, velocity
    """
    pose = self.position(world)
    if pose is None:
      return None
    velocity = self.velocity()
    return np.array([0, pose[0], pose[1], pose[2], velocity])

  def ds(self):
    """Increment for placing the agents
    
    Keyword Arguments:
        s_min {float} -- Min. lon. distance (default: {5.})
        s_max {float} -- Max. lon. distance (default: {10.})
    
    Returns:
        float -- delta s-value
    """
    return np.random.uniform(self._ds_min, self._ds_max)

  def position(self, world):
    """Using the defined LaneCorridor it finds positions for the agents
    
    Arguments:
        world {BARK.world} -- BARK world
    
    Keyword Arguments:
        min_s {float} -- Min. lon. value (default: {0.})
        max_s {float} -- Max. lon. value (default: {100.})
    
    Returns:
        tuple -- (x, y, theta)
    """
    if self._road_corridor == None:
      world.map.GenerateRoadCorridor(
        self._road_ids, XodrDrivingDirection.forward)
      self._road_corridor = world.map.GetRoadCorridor(
        self._road_ids, XodrDrivingDirection.forward)
    if self._road_corridor is None:
      return None
    if self._lane_corridor:
      lane_corr = self._lane_corridor
    else:
      lane_corr = self._road_corridor.lane_corridors[self._lane_corridor_id]
    if lane_corr is None:
      return None
    centerline = lane_corr.center_line
    if self._current_s == None:
      self._current_s = self._s_min
    xy_point =  GetPointAtS(centerline, self._current_s)
    angle = GetTangentAngleAtS(centerline, self._current_s)
    if self._current_s > self._s_max:
      return None
    self._current_s += self.ds()
    return (xy_point.x(), xy_point.y(), angle)

  def velocity(self):
    return np.random.uniform(low=self._min_vel, high=self._max_vel)

  def behavior_model(self, world):
    """Returns behavior model
    """
    return self._behavior_model

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
    return Polygon2d([0, 0, 0], [Point2d(-2, -1),
                      Point2d(2, -1),
                      Point2d(2, 1),
                      Point2d(-2, 1),
                      Point2d(-2, -1)])

  def goal(self, world):
    """Returns goal def.
    """
    # TODO: by default should be based on agent's pos
    road_corr = world.map.GetRoadCorridor(
      self._road_ids, XodrDrivingDirection.forward)
    if self._lane_corridor:
      lane_corr = self._lane_corridor
    else:
      lane_corr = self._road_corridor.lane_corridors[self._lane_corridor_id]
    # TODO: check
    return GoalDefinitionStateLimitsFrenet(lane_corr.center_line,
                                           (0.2, 0.2),
                                           (0.1, 0.1),
                                           (10., 15.))

  def controlled_ids(self, agent_list):
    """Returns an ID-List of controlled agents
    """
    if self._controlled_ids is None:
      return []
    random_int = [agent_list[np.random.randint(0, len(agent_list))]]
    return random_int

  def controlled_goal(self, world):
    """Goal for the controlled agent
    
    Arguments:
        world {BARK.world} -- BARK world
    
    Returns:
        GoalDefinition -- Goal for the controlled agent
    """
    return self.goal(world)

  def controlled_behavior_model(self, world):
    """Behavior model for controlled agent
    
    Returns:
        BehaviorModel -- BARK behavior model
    """
    if self._controlled_behavior_model is None:
      return self.behavior_model(world)

  def reset(self):
    """Resets the LaneCorridorConfig
    """
    self._current_s = None


class ConfigWithEase(ScenarioGeneration):
  """Configure your scenarios with ease
     &copy; Patrick Hart
  """
  def __init__(self,
               num_scenarios,
               map_file_name=None,
               params=None,
               random_seed=None,
               lane_corridor_configs=None):
    self._map_file_name = map_file_name
    self._lane_corridor_configs = lane_corridor_configs or []
    super(ConfigWithEase, self).__init__(params, num_scenarios)
    self.initialize_params(params)

  def create_scenarios(self, params, num_scenarios):
    """see baseclass
    """
    scenario_list = []
    for scenario_idx in range(0, num_scenarios):
      scenario = self.create_single_scenario()
      scenario_list.append(scenario)
    return scenario_list

  def create_single_scenario(self):
    """Creates one scenario using the defined LaneCorridorConfig
    
    Returns:
        Scenario -- Returns a BARK scenario
    """
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()
    map_interface = world.map
    # fill agent list of the BARK world and set agents that are controlled
    scenario._agent_list = []
    scenario._eval_agent_ids = []
    for lc_config in self._lane_corridor_configs:
      agent_state = True
      lc_agents = []
      if lc_config._source_pos is not None and lc_config._sink_pos is not None:
        lc_config.InferRoadIdsAndLaneCorr(world)
      while agent_state is not None:
        agent_state = lc_config.state(world)
        if agent_state is not None:
          agent_behavior = lc_config.behavior_model(world)
          agent_dyn = lc_config.dynamic_model
          agent_exec = lc_config.execution_model
          agent_polygon = lc_config.shape
          agent_params = self._params.addChild("agent")
          agent_goal = lc_config.goal(world)
          new_agent = Agent(
            agent_state, 
            agent_behavior, 
            agent_dyn,
            agent_exec, 
            agent_polygon,
            agent_params,
            agent_goal,
            map_interface)
          new_agent.road_corridor = lc_config._road_corridor
          lc_agents.append(new_agent)
        # set the road corridor

      # handle controlled agents
      controlled_agent_ids = []
      for controlled_agent in lc_config.controlled_ids(lc_agents):
        controlled_agent.goal_definition = lc_config.controlled_goal(world)
        controlled_agent.behavior_model = \
          lc_config.controlled_behavior_model(world)
        controlled_agent_ids.append(controlled_agent.id)
      scenario._eval_agent_ids.extend(controlled_agent_ids)
      scenario._agent_list.extend(lc_agents)
      lc_config.reset()
    return scenario