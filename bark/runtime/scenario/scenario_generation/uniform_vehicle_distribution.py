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
from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.runtime.commons import ParameterServer
from bark.runtime.commons import XodrParser

import numpy as np
import math


class UniformVehicleDistribution(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(UniformVehicleDistribution, self).__init__(params, num_scenarios)
    self.initialize_params(params)

  def initialize_params(self, params):
    params_temp = \
      self._params["Scenario"]["Generation"]["UniformVehicleDistribution"]
    self._random_seed = 1000 # since ScenarioGeneration UniformVehicleDistribution
      # will soon be deprecated we introduce this hack
    self._map_file_name = params_temp["MapFilename",
      "Path to the open drive map", 
      "bark/runtime/tests/data/city_highway_straight.xodr",    ]
    self._ego_goal_end = params_temp["EgoGoalEnd",
      "The center of the ego agent's goal region polygon",
      [4.317, 21] ]
    self._ego_goal_start = params_temp["EgoGoalStart",
      "The coordinates of the start of the ego goal,\
           if empty only ego goal end is used as center of polygon ",
      [] ]
    self._ego_goal_state_limits = params_temp["EgoGoalStateLimits",
      "x,y and theta limits around center line of lane between start and end applied to both lateral sides \
       (only valid if start and end goal of ego are given)",
       [0.1, 0, 0.08]]
    self._ego_route = params_temp["EgoRoute",
      "A list of two points defining start and end point of initial ego driving corridor. \
           If empty, then one of the other agents is selected as ego agents.",
      [[4.817, -79], [4.817, 21]]]
    self._others_source = params_temp["OthersSource",
      "A list of points around which other vehicles spawn. \
        Points should be on different lanes. XodrLanes must be near these points \
      (<0.5m) Provide a list of lists with x,y-coordinates",
     [[1.943, -117.1695]]]
    self._others_sink = params_temp["OthersSink",
      "A list of points defining end of other vehicles routes.\
        Points should be on different lanes and match the order of the\
        source points. XodrLanes must be near these points (<0.5m) \
        Provide a list of lists with x,y-coordinates",
        [[ 1.943, 14.1725]] ]  
    assert len(self._others_sink) == len(self._others_source)         
    self._vehicle_distance_range = params_temp["VehicleDistanceRange",
      "Distance range between vehicles in meter given as tuple from which" + \
      "distances are sampled uniformly",
      (10, 20)]
    self._other_velocity_range = params_temp["OtherVehicleVelocityRange",
      "Lower and upper bound of velocity in km/h given as tuple from which" + \
      " velocities are sampled uniformly",
      (20,30)]
    self._ego_velocity_range = params_temp["EgoVehicleVelocityRange",
      "Lower and upper bound of velocity in km/h given as tuple from which" + \
      " velocities are sampled uniformly",
      (20,30)]
    json_converter = ModelJsonConversion()
    self._agent_params = params_temp["VehicleModel",
      "How to model the other agents",
      json_converter.agent_to_json(self.default_agent_model())]
    if not isinstance(self._agent_params, dict):
        self._agent_params = self._agent_params.ConvertToDict()
    np.random.seed(self._random_seed)


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
                        json_params=self._params.ConvertToDict())
    world = scenario.GetWorldState()
    agent_list = []
    # OTHER AGENTS
    for idx, source in enumerate(self._others_source):
      connecting_center_line, s_start, s_end = \
        self.center_line_between_source_and_sink(world.map,
                                                 source,
                                                 self._others_sink[idx])
      goal_polygon = Polygon2d([0, 0, 0],
                               [Point2d(-1.5,0),
                                Point2d(-1.5,8),
                                Point2d(1.5,8),
                                Point2d(1.5,0)])
      goal_polygon = goal_polygon.Translate(Point2d(self._others_sink[idx][0],
                                                    self._others_sink[idx][1]))
      goal_definition = GoalDefinitionPolygon(goal_polygon)
      agent_list.extend(
        self.place_agents_along_linestring(world,
                                           connecting_center_line,
                                           s_start,
                                           s_end,
                                           self._agent_params,
                                           goal_definition))

    description=self._params.ConvertToDict()
    description["ScenarioGenerator"] = "UniformVehicleDistribution"
    

    # EGO AGENT
    ego_agent=None
    if len(self._ego_route) == 0:
        # take agent in the middle of list 
        num_agents = len(agent_list)
        ego_agent = agent_list[math.floor(num_agents/4)] 
    else:
        connecting_center_line, s_start, s_end  = \
        self.center_line_between_source_and_sink(world.map,
                                                 self._ego_route[0],
                                                 self._ego_route[1])

        sego = self.sample_srange_uniform([s_start, s_end])
        xy_point =  GetPointAtS(connecting_center_line, sego)
        angle = GetTangentAngleAtS(connecting_center_line, sego)
        velocity = self.sample_velocity_uniform(self._ego_velocity_range)
        agent_state = np.array([0, xy_point.x(), xy_point.y(), angle, velocity ])

        agent_params = self._agent_params.copy()
        agent_params["state"] = agent_state
        # goal for driving corridor generation
        goal_polygon = Polygon2d([0, 0, 0],
                               [Point2d(-1.5,0),
                                Point2d(-1.5,8),
                                Point2d(1.5,8),
                                Point2d(1.5,0)])
        goal_polygon = goal_polygon.Translate(Point2d(self._ego_route[1][0],
                                                    self._ego_route[1][1]))
        goal_definition = GoalDefinitionPolygon(goal_polygon)
        agent_params["goal_definition"] = goal_definition
        agent_params["map_interface"] = world.map

        converter = ModelJsonConversion()
        ego_agent = converter.agent_from_json(agent_params, self._params["Agent"])
        # TODO(@bernhard): ensure that ego agent not collides with others
    
    agent_list.append(ego_agent)


    
    # EGO Agent Goal Definition
    if  len(self._ego_goal_start) == 0:
        if len(self._ego_route) == 0:
          # ego agent is one of the random agents, so the goal definition is
          # already set
          pass
        else:
          goal_polygon = Polygon2d([0, 0, 0],
                                   [Point2d(-1.5,0),
                                    Point2d(-1.5,8),
                                    Point2d(1.5,8),
                                    Point2d(1.5,0)])
          goal_polygon = goal_polygon.Translate(Point2d(self._ego_goal_end[0],
                                                      self._ego_goal_end[1]))
          ego_agent.goal_definition = GoalDefinitionPolygon(goal_polygon)
    else:
        connecting_center_line, s_start, s_end = \
        self.center_line_between_source_and_sink(world.map,
                                                 self._ego_goal_start,
                                                 self._ego_goal_end)

        goal_center_line = GetLineFromSInterval(connecting_center_line, s_start, s_end)

        # build polygon representing state limits
        lims = self._ego_goal_state_limits
        goal_limits_left = goal_center_line.Translate(Point2d(-lims[0], -lims[1]))
        goal_limits_right = goal_center_line.Translate(Point2d(lims[0], lims[1]))
        goal_limits_right.Reverse()
        goal_limits_left.AppendLinestring(goal_limits_right)
        polygon = Polygon2d([0,0,0], goal_limits_left)

        ego_agent.goal_definition = GoalDefinitionStateLimits(polygon, (1.57-0.08, 1.57+0.08))

    # only one agent is ego in the middle of all other agents
    scenario._agent_list = agent_list
    scenario._eval_agent_ids = [ego_agent.id]
    return scenario

  def place_agents_along_linestring(self,
                                    world,
                                    linestring,
                                    s_start,
                                    s_end,
                                    agent_params,
                                    goal_definition):
    s = s_start
    if s_end < s_start:
      linestring.Reverse()
      s = s_end
      s_end = s_start
    agent_list = []
    while s < s_end:
      # set agent state on linestring with random velocity
      xy_point =  GetPointAtS(linestring, s)
      angle = GetTangentAngleAtS(linestring, s)
      
      velocity = self.sample_velocity_uniform(self._other_velocity_range)
      agent_state = np.array([0, xy_point.x(), xy_point.y(), angle, velocity ])

      agent_params = self._agent_params.copy()
      agent_params["state"] = agent_state
      agent_params["goal_definition"] = goal_definition
      agent_params["map_interface"] = world.map

      converter = ModelJsonConversion()
      bark_agent = converter.agent_from_json(agent_params, self._params["Agent"])
      agent_list.append(bark_agent)

      # move forward on linestring based on vehicle size and max/min distance
      s += bark_agent.shape.front_dist + bark_agent.shape.rear_dist + \
                  self.sample_distance_uniform(self._vehicle_distance_range)
    return agent_list


  def sample_velocity_uniform(self, velocity_range):
    return np.random.uniform(velocity_range[0], velocity_range[1])

  def sample_distance_uniform(self, distance_range):
    return np.random.uniform(distance_range[0], distance_range[1])

  def sample_srange_uniform(self, srange):
    return np.random.uniform(srange[0], srange[1])

  def center_line_between_source_and_sink(self, map_interface, source, sink):
    # generate road corridor between source and sink
    goal_polygon = Polygon2d([0, 0, 0],
                                  [Point2d(-1,0),
                                  Point2d(-1,1),
                                  Point2d(1,1),
                                  Point2d(1,0)])
    goal_polygon = goal_polygon.Translate(Point2d(sink[0],
                                                      sink[1]))
    road_corridor = map_interface.GenerateRoadCorridor(
        Point2d(source[0], source[1]),
        goal_polygon)

    # find lane corridor between source and sink
    lane_corridor_sink = road_corridor.GetCurrentLaneCorridor(Point2d(sink[0],
                                                      sink[1]))
    lane_corridor_source = road_corridor.GetCurrentLaneCorridor(Point2d(source[0],
                                                      source[1]))

    if (not lane_corridor_sink or not lane_corridor_source) or  lane_corridor_sink != lane_corridor_source:
      print("source and sink lane corridors not equal!.")
      return

    center_line = lane_corridor_sink.center_line

    _, s_start, _ = GetNearestPointAndS(center_line,
                                            Point2d(source[0],source[1]))
    _, s_end, _ = GetNearestPointAndS(center_line,
                                          Point2d(sink[0],sink[1]))
    return center_line, \
           s_start, \
           s_end


  def default_agent_model(self):
    param_server = ParameterServer()
    behavior_model = BehaviorConstantAcceleration(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)
    map_interface = MapInterface()

    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])

    agent_default = Agent(init_state,
                          behavior_model,
                          dynamic_model,
                          execution_model,
                          agent_2d_shape,
                          param_server)

    return agent_default
