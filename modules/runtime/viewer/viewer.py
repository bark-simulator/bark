# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import logging
from bark.viewer import Viewer
from bark.geometry import *
from bark.models.dynamic import *
from bark.world.opendrive import *
from bark.world.goal_definition import *
from modules.runtime.commons.parameters import ParameterServer

logger = logging.getLogger()

class BaseViewer(Viewer):
    def __init__(self, params=None, **kwargs):
        if(params is None):
           params = ParameterServer()
        Viewer.__init__(self)
        # color parameters
        # agents
        self.color_other_agents = params["Visualization"]["Agents"]["Color"]["Other", "Color of other agents", (0.7,0.7,0.7)]
        self.color_eval_agents = params["Visualization"]["Agents"]["Color"]["Controlled", "Color of controlled, evaluated agents", (0.9,0,0)]
        self.alpha_agents = params["Visualization"]["Agents"]["AlphaVehicle", "Alpha of agents", 0.8]
        self.route_color =  params["Visualization"]["Agents"]["ColorRoute", "Color of agents routes", (0.2,0.2,0.2)]
        self.draw_route = params["Visualization"]["Agents"]["DrawRoute", "Draw Route of each agent", False]
        self.draw_eval_goals = params["Visualization"]["Agents"]["DrawEvalGoals", "Draw Route of eval agent goals", True]
        self.eval_goal_color = params["Visualization"]["Agents"]["EvalGoalColor", "Color of eval agent goals", (0.0,0.0,0.7)]
        self.draw_history = params["Visualization"]["Agents"]["DrawHistory", "Draw history with alpha trace for each agent", True]
        # map
        self.color_lane_boundaries = params["Visualization"]["Map"]["Lanes"]["Boundaries"]["Color", "Color of agents except ego vehicle", (0.7,0.7,0.7)]
        self.alpha_lane_boundaries = params["Visualization"]["Map"]["Lanes"]["Boundaries"]["Alpha", "Color of agents except ego vehicle", 1.0]
        self.plane_color = params["Visualization"]["Map"]["Plane"]["Color", "Color of the background plane", (1, 1, 1, 1)]
        self.plane_alpha = params["Visualization"]["Map"]["Plane"]["Alpha", "Alpha of the background plane", 1.0]


        self.parameters = params

        self.world_x_range = kwargs.pop("x_range", np.array([-40, 40]))
        self.world_y_range = kwargs.pop("y_range", np.array([-40, 40]))
        self.use_world_bounds = kwargs.pop("use_world_bounds", False)
        self.follow_agent_id = kwargs.pop("follow_agent_id", None)

        self.dynamic_world_x_range = self.world_x_range.copy()
        self.dynamic_world_y_range = self.world_y_range.copy()

    def reset():
        pass

    def _get_draw_eval_agent_ids(self, world, eval_agent_ids=None, ):
        if self.follow_agent_id is not None:
            if isinstance(self.follow_agent_id, bool) and \
                     eval_agent_ids is not None and \
                     len(eval_agent_ids) == 1:
                draw_eval_agent_id = eval_agent_ids[0]
            else:
                draw_eval_agent_id = self.follow_agent_id

            if draw_eval_agent_id in world.agents:
                return draw_eval_agent_id

        return None

    def _update_world_view_range(self, world, eval_agent_ids=None):
        draw_eval_agent_id = self._get_draw_eval_agent_ids(world, eval_agent_ids)

        if draw_eval_agent_id != None:
            follow_agent = world.agents[draw_eval_agent_id]
            state = follow_agent.state
            pose = np.zeros(3)
            pose[0] = state[int(StateDefinition.X_POSITION)]
            pose[1] = state[int(StateDefinition.Y_POSITION)]
            pose[2] = state[int(StateDefinition.THETA_POSITION)]

            # center range on agents coordinates
            self.dynamic_world_x_range[0] = pose[0] + self.world_x_range[0]
            self.dynamic_world_x_range[1] = pose[0] + self.world_x_range[1]

            self.dynamic_world_y_range[0] = pose[1] + self.world_y_range[0]
            self.dynamic_world_y_range[1] = pose[1] + self.world_y_range[1]

        if self.use_world_bounds:
            bb = world.bounding_box
            self.dynamic_world_x_range = [bb[0].x(), bb[1].x()]
            self.dynamic_world_y_range = [bb[0].y(), bb[1].y()]

            diffx = abs(self.dynamic_world_x_range[1] - self.dynamic_world_x_range[0])
            diffy = abs(self.dynamic_world_y_range[1] - self.dynamic_world_y_range[0])

            # enforce that in both dimensions  the same range is covered
            if diffx > diffy:
                self.dynamic_world_y_range[0] -= (diffx - diffy)/2
                self.dynamic_world_y_range[1] += (diffx - diffy)/2
            else:
                self.dynamic_world_x_range[0] -= (diffy - diffx)/2
                self.dynamic_world_x_range[1] += (diffy - diffx)/2

    def drawPoint2d(self, point2d, color, alpha):
        pass

    def drawLine2d(self, line2d, color, alpha, line_style=None):
        pass

    def drawPolygon2d(self, polygon, color, alpha):
        pass

    def drawTrajectory(self, trajectory, color):
        pass

    def drawObstacle(self, obstacle):
        pass

    def drawText(self, position, text, **kwargs):
        pass

    def getColor(self, color):
        pass

    def show(self,block=False):
        pass

    def clear(self):
        pass

    def drawAgents(self, world):
        for _, agent in world.agents.items():
            self.drawAgent(agent)

    def drawHistory(self, agent, color):
        shape = agent.shape
        if isinstance(shape, Polygon2d):
            history = agent.history
            lh = len(history)
            for idx, state_action in enumerate(history):
                state = state_action[0]
                pose = np.zeros(3)
                # pybind creates column based vectors, initialization maybe row-based -> we consider both
                pose[0] = state[int(StateDefinition.X_POSITION)]
                pose[1] = state[int(StateDefinition.Y_POSITION)]
                pose[2] = state[int(StateDefinition.THETA_POSITION)]
                transformed_polygon = shape.transform(pose)
                alpha=1-0.8*(lh-idx)/4
                alpha = 0 if alpha<0 else alpha
                self.drawPolygon2d(transformed_polygon, color, alpha) # fade to 0.2 after 10 steps
    
    def drawGoalDefinition(self, goal_definition, color="blue"):
        if isinstance(goal_definition, GoalDefinitionPolygon):
            self.drawPolygon2d(goal_definition.goal_shape, color, alpha=0.1)
        elif isinstance(goal_definition, GoalDefinitionStateLimits):
            self.drawPolygon2d(goal_definition.xy_limits, color, alpha=0.1)
        elif isinstance(goal_definition, GoalDefinitionSequential):
            prev_center = np.array([])
            for idx, goal_def in enumerate(goal_definition.sequential_goals):
                self.drawGoalDefinition(goal_def, color=color)
                goal_pos = None
                if isinstance(goal_def, GoalDefinitionPolygon):
                    goal_pos = goal_def.goal_shape.center
                elif isinstance(goal_def, GoalDefinitionStateLimits):
                    goal_pos = goal_def.xy_limits.center
                # self.drawText(position=goal_pos, text="Goal{}".format(idx), coordinate="world")
                if prev_center.any():
                    line = Line2d()
                    line.addPoint(Point2d(prev_center[0], prev_center[1]))
                    line.addPoint(Point2d(goal_pos[0], goal_pos[1]))
                    self.drawLine2d(line, color, alpha=0.9)
                prev_center = goal_pos

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None):
        self.clear()
        self._update_world_view_range(world, eval_agent_ids)
        if world.map:
            self.drawMap(world.map.get_open_drive_map())

        # draw agents
        for agent_id, agent in world.agents.items():
            # TODO(@hart): draw agents and goals in the same color
            #              support mult. eval. agents and goals
            if self.draw_eval_goals and agent.goal_definition:
                color = self.eval_goal_color
                try:
                  color = tuple(
                    self.parameters["Scenario"]["ColorMap"][str(agent_id), "color", [1., 0., 0.]])
                except:
                  pass
                self.drawGoalDefinition(agent.goal_definition, color)
            
        for agent_id, agent in world.agents.items():
            color = "blue"
            if eval_agent_ids and agent.id in eval_agent_ids:
                color = self.color_eval_agents
                try:
                  color = tuple(
                    self.parameters["Scenario"]["ColorMap"][str(agent_id), "color", [1., 0., 0.]])
                except:
                  pass
            else:
                color = self.color_other_agents
            self.drawAgent(agent, color)

        self.drawText(position=(0.1,0.9), text="Scenario: {}".format(scenario_idx), fontsize=18)
        self.drawText(position=(0.1,0.95), text="Time: {:.2f}".format(world.time), fontsize=18)

    def drawMap(self, map):
        # draw the boundary of each lane
        for _, road in map.get_roads().items():
            self.drawRoad(road, self.color_lane_boundaries)

    def drawRoad(self, road, color=None):
        for lane_section in road.lane_sections:
          self.drawLaneSection(lane_section, color)
    
    def drawLaneSection(self, lane_section, color=None):
      for _, lane in lane_section.get_lanes().items():
        self.drawLane(lane, color)
        
    def drawLane(self, lane, color=None):
      if color is None:
        self.color_lane_boundaries

      dashed = False
      # center line is type none and is drawn as broken
      if lane.road_mark.type == RoadMarkType.broken or lane.road_mark.type == RoadMarkType.none: 
        dashed = True
      self.drawLine2d(lane.line, color, self.alpha_lane_boundaries, dashed)

    def drawAgent(self, agent, color):
        shape = agent.shape
        if isinstance(shape, Polygon2d):
            pose = np.zeros(3)
            # pybind creates column based vectors, initialization maybe row-based -> we consider both
            state = agent.state
            pose[0] = state[int(StateDefinition.X_POSITION)]
            pose[1] = state[int(StateDefinition.Y_POSITION)]
            pose[2] = state[int(StateDefinition.THETA_POSITION)]
            transformed_polygon = shape.transform(pose)
            self.drawPolygon2d(transformed_polygon, color, 1.0)

        if self.draw_route:
            self.drawRoute(agent)

        # self.drawHistory(agent, color)

    def drawDrivingCorridor(self, corridor, color=None):
        if color is None:
            # generate random colour
            color = list(np.random.choice(range(256), size=3)/256)
        if corridor.center.valid() and corridor.inner.valid() and corridor.outer.valid():
            self.drawLine2d(corridor.center, color, 1, True)
            self.drawLine2d(corridor.inner, color, 1)
            self.drawLine2d(corridor.outer, color, 1)
        else:
            logger.info("Cannot draw Driving Corridor, as it is empty")        

    def drawRoute(self, agent):
        # TODO(@hart): visualize the global as well as the local driving corridor
        self.drawDrivingCorridor(agent.local_map.get_driving_corridor(), self.route_color)
        self.drawDrivingCorridor(agent.local_map.get_horizon_driving_corridor(), (0.8, 0.72, 0.2))
