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
               road_ids=None,
               lane_corridor_id=0,
               params=None):
    self._road_corridor = None
    self._road_ids = road_ids
    self._lane_corridor_id = lane_corridor_id
    self._params = params
    self._current_s = None
  
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

  def ds(self, s_min=5., s_max=10.):
    """Increment for placing the agents
    
    Keyword Arguments:
        s_min {float} -- Min. lon. distance (default: {5.})
        s_max {float} -- Max. lon. distance (default: {10.})
    
    Returns:
        float -- delta s-value
    """
    return np.random.uniform(s_min, s_max)

  def position(self, world, min_s=0., max_s=100.):
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
      world.map.GenerateRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
    road_corr = world.map.GetRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
    if road_corr is None:
      return None
    lane_corr = road_corr.lane_corridors[self._lane_corridor_id]
    if lane_corr is None:
      return None
    centerline = lane_corr.center_line
    if self._current_s == None:
      self._current_s = min_s
    xy_point =  GetPointAtS(centerline, self._current_s)
    angle = GetTangentAngleAtS(centerline, self._current_s)
    if self._current_s > max_s:
      return None
    self._current_s += self.ds()
    return (xy_point.x(), xy_point.y(), angle)

  def velocity(self, min_vel=10., max_vel=15.):
    return np.random.uniform(low=min_vel, high=max_vel)

  @property
  def behavior_model(self):
    """Returns behavior model
    """
    return BehaviorIDMClassic(self._params)

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
    # should be access safe, otherwise would not reach.
    road_corr = world.map.GetRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
    lane_corr = road_corr.lane_corridors[self._lane_corridor_id]
    return GoalDefinitionStateLimitsFrenet(lane_corr.center_line,
                                           (0.2, 0.2),
                                           (0.1, 0.1),
                                           (10., 15.))

  def controlled_ids(self, agent_list):
    """Returns an ID-List of controlled agents
    """
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

  @property
  def controlled_behavior_model(self):
    """Behavior model for controlled agent
    
    Returns:
        BehaviorModel -- BARK behavior model
    """
    return self.behavior_model

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
      while agent_state is not None:
        agent_state = lc_config.state(world)
        if agent_state is not None:
          agent_behavior = lc_config.behavior_model
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
          lc_agents.append(new_agent)
      # handle controlled agents
      controlled_agent_ids = []
      for controlled_agent in lc_config.controlled_ids(lc_agents):
        controlled_agent.goal_definition = lc_config.controlled_goal(world)
        controlled_agent.behavior_model = lc_config.controlled_behavior_model
        controlled_agent_ids.append(controlled_agent.id)
      scenario._eval_agent_ids.extend(controlled_agent_ids)
      scenario._agent_list.extend(lc_agents)
      lc_config.reset()
    return scenario