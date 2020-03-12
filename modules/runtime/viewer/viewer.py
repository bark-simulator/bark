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
import math

logger = logging.getLogger()

class BaseViewer(Viewer):
    def __init__(self, params=None, **kwargs):
        if(params is None):
           params = ParameterServer()
        Viewer.__init__(self)
        # color parameters
        # agents
        self.color_other_agents_line = params["Visualization"]["Agents"]["Color"]["Other"]["Lines", "Color of other agents", (0.7,0.7,0.7)]
        self.color_other_agents_face = params["Visualization"]["Agents"]["Color"]["Other"]["Face", "Color of other agents", (0.7,0.7,0.7)]
        self.color_eval_agents_line = params["Visualization"]["Agents"]["Color"]["Controlled"]["Lines", "Color of controlled, evaluated agents", (0.9,0,0)]
        self.color_eval_agents_face = params["Visualization"]["Agents"]["Color"]["Controlled"]["Face", "Color of controlled, evaluated agents", (0.9,0,0)]
        self.use_colormap_for_other_agents = params["Visualization"]["Agents"]["Color"]["UseColormapForOtherAgents", "Flag to enable color map for other agents", True]
        self.alpha_eval_agent = params["Visualization"]["Agents"]["Alpha"]["Controlled", "Alpha of evalagents", 0.8]
        self.alpha_other_agents = params["Visualization"]["Agents"]["Alpha"]["Other", "Alpha of other agents", 1]
        self.route_color =  params["Visualization"]["Agents"]["ColorRoute", "Color of agents routes", (0.2,0.2,0.2)]
        self.draw_route = params["Visualization"]["Agents"]["DrawRoute", "Draw Route of each agent", False]
        self.draw_agent_id = params["Visualization"]["Agents"]["DrawAgentId", "Draw id of each agent", False]
        self.draw_eval_goals = params["Visualization"]["Agents"]["DrawEvalGoals", "Draw Route of eval agent goals", True]
        self.eval_goal_color = params["Visualization"]["Agents"]["EvalGoalColor", "Color of eval agent goals", (0.0,0.0,0.7)]
        self.draw_history = params["Visualization"]["Agents"]["DrawHistory", "Draw history with alpha trace for each agent", True]
        # map
        self.color_lane_boundaries = params["Visualization"]["Map"]["XodrLanes"]["Boundaries"]["Color", "Color of agents except ego vehicle", (0.7,0.7,0.7)]
        self.alpha_lane_boundaries = params["Visualization"]["Map"]["XodrLanes"]["Boundaries"]["Alpha", "Color of agents except ego vehicle", 1.0]
        self.plane_color = params["Visualization"]["Map"]["Plane"]["Color", "Color of the background plane", (1, 1, 1, 1)]
        self.plane_alpha = params["Visualization"]["Map"]["Plane"]["Alpha", "Alpha of the background plane", 1.0]

        self.parameters = params

        self.use_world_bounds = kwargs.pop("use_world_bounds", False)
        self.follow_agent_id = kwargs.pop("follow_agent_id", None)

        self.center = kwargs.pop("center", np.array([0, 0]))

        self.world_x_range = kwargs.pop("x_range", np.array([-40, 40]))
        self.world_y_range = kwargs.pop("y_range", np.array([-40, 40]))

        self.enforce_x_length = kwargs.pop("enforce_x_length", True)
        self.enforce_y_length = kwargs.pop("enforce_y_length", False)
        self.x_length = kwargs.pop("x_length", np.sum(np.absolute(self.world_x_range)))
        self.y_length = kwargs.pop("y_length", np.sum(np.absolute(self.world_y_range)))

        self.dynamic_world_x_range = self.world_x_range.copy()
        self.dynamic_world_y_range = self.world_y_range.copy()

    def reset():
        pass

    def get_aspect_ratio(self):
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

            center = [pose[0],  pose[1]]
            self._update_world_dynamic_range(center)
        else:
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
          else:
              center = self.center
              self._update_world_dynamic_range(center)


    def _update_world_dynamic_range(self, center):
        aspect_ratio = self.get_aspect_ratio()
        if self.enforce_x_length:
            self.dynamic_world_x_range = [-self.x_length/2 + center[0], self.x_length/2 + center[0]]
            self.dynamic_world_y_range = [-self.x_length/2/aspect_ratio + center[1], self.x_length/2/aspect_ratio + center[1]]

        if self.enforce_y_length:
            self.dynamic_world_x_range = [-self.y_length/2*aspect_ratio + center[0], self.y_length/2*aspect_ratio + center[0]]
            self.dynamic_world_y_range = [-self.y_length/2 + center[1], self.y_length/2 + center[1]]

    def drawPoint2d(self, point2d, color, alpha):
        pass

    def drawLine2d(self, line2d, color, alpha, line_style=None, zorder=10):
        pass

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None):
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

    def drawHistory(self, agent, color, alpha, facecolor):
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
                transformed_polygon = shape.Transform(pose)
                alpha=1-0.8*(lh-idx)/4
                alpha = 0 if alpha<0 else alpha
                self.drawPolygon2d(transformed_polygon, color, alpha, facecolor) # fade to 0.2 after 10 steps
    
    def drawGoalDefinition(self, goal_definition, color, alpha, facecolor):
        if isinstance(goal_definition, GoalDefinitionPolygon):
            self.drawPolygon2d(goal_definition.goal_shape, color, alpha, facecolor)
        elif isinstance(goal_definition, GoalDefinitionStateLimits):
            self.drawPolygon2d(goal_definition.xy_limits, color, alpha, facecolor)
        elif isinstance(goal_definition, GoalDefinitionStateLimitsFrenet):
            self.drawPolygon2d(goal_definition.goal_shape, color, alpha, facecolor)
        elif isinstance(goal_definition, GoalDefinitionSequential):
            prev_center = np.array([])
            for idx, goal_def in enumerate(goal_definition.sequential_goals):
                self.drawGoalDefinition(goal_def, color, alpha, facecolor)
                goal_pos = None
                if isinstance(goal_def, GoalDefinitionPolygon):
                    goal_pos = goal_def.goal_shape.center
                elif isinstance(goal_def, GoalDefinitionStateLimits):
                    goal_pos = goal_def.xy_limits.center
                # self.drawText(position=goal_pos, text="Goal{}".format(idx), coordinate="world")
                if prev_center.any():
                    line = Line2d()
                    line.AddPoint(Point2d(prev_center[0], prev_center[1]))
                    line.AddPoint(Point2d(goal_pos[0], goal_pos[1]))
                    self.drawLine2d(line, color, alpha=0.9)
                prev_center = goal_pos

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
        # self.clear()
        self._update_world_view_range(world, eval_agent_ids)
        if world.map:
            self.drawMap(world.map.GetOpenDriveMap())

        # draw agent goals
        for agent_id, agent in world.agents.items():
            if eval_agent_ids and self.draw_eval_goals and agent.goal_definition and \
                    agent_id == eval_agent_ids[0]:
                color_line = self.color_eval_agents_line
                color_face = self.color_eval_agents_face
                alpha = self.alpha_eval_agent
                self.drawGoalDefinition(agent.goal_definition, color_line, alpha, color_face)

        num_agents = len(world.agents.items())
        for i, (agent_id, agent) in enumerate(world.agents.items()):
            color = "blue"
            alpha = 1.0
            if eval_agent_ids and agent.id in eval_agent_ids:
                color_line = self.color_eval_agents_line
                color_face = self.color_eval_agents_face
                alpha = self.alpha_eval_agent
            else:
                alpha = self.alpha_other_agents
                if self.use_colormap_for_other_agents:
                  color_line = self.getColorFromMap(float(i) / float(num_agents))
                  color_face = self.getColorFromMap(float(i) / float(num_agents))
                else:
                  color_line = self.color_other_agents_line
                  color_face = self.color_other_agents_face
            self.drawAgent(agent, color_line, alpha, color_face)
            if self.drawHistory:
                self.drawHistory(agent, color_line, alpha, color_face)
        if debug_text:
          self.drawText(position=(0.1, 0.9), text="Scenario: {}".format(scenario_idx), fontsize=14)
          self.drawText(position=(0.1, 0.95), text="Time: {:.2f}".format(world.time), fontsize=14)
      

    def drawMap(self, map):
        # draw the boundary of each lane
        for _, road in map.GetRoads().items():
            self.drawXodrRoad(road, self.color_lane_boundaries)

    def drawXodrRoad(self, road, color=None):
        for lane_section in road.lane_sections:
          self.drawXodrLaneSection(lane_section, color)
    
    def drawXodrLaneSection(self, lane_section, color=None):
      for _, lane in lane_section.GetLanes().items():
        self.drawXodrLane(lane, color)
        
    def drawXodrLane(self, lane, color=None):
      if color is None:
        self.color_lane_boundaries

      dashed = False
      # center line is type none and is drawn as broken
      if lane.road_mark.type == XodrRoadMarkType.broken or lane.road_mark.type == XodrRoadMarkType.none: 
        dashed = True
      self.drawLine2d(lane.line, color, self.alpha_lane_boundaries, dashed, zorder=2)

    def drawAgent(self, agent, color, alpha, facecolor):
        shape = agent.shape
        if isinstance(shape, Polygon2d):
            pose = np.zeros(3)
            # pybind creates column based vectors, initialization maybe row-based -> we consider both
            state = agent.state
            pose[0] = state[int(StateDefinition.X_POSITION)]
            pose[1] = state[int(StateDefinition.Y_POSITION)]
            pose[2] = state[int(StateDefinition.THETA_POSITION)]
            transformed_polygon = shape.Transform(pose)

            centerx = (shape.front_dist - 0.5*(shape.front_dist+shape.rear_dist)) * math.cos(pose[2]) + pose[0]
            centery = (shape.front_dist - 0.5*(shape.front_dist+shape.rear_dist))* math.sin(pose[2]) + pose[1]

            if self.draw_agent_id:
              self.drawText(position=(centerx, centery), rotation=180.0*(1.0+pose[2]/math.pi), text="{}".format(agent.id),\
                 coordinate="not axes", ha='center', va="center", multialignment="center", size="smaller")
            
            self.drawPolygon2d(transformed_polygon, color, alpha, facecolor)
        else:
            raise NotImplementedError("Shape drawing not implemented.")

    def drawLaneCorridor(self, lane_corridor):
      self.drawPolygon2d(lane_corridor.polygon, color="blue", alpha=.5)

    def drawRoadCorridor(self, road_corridor):
      # TODO(@hart): use agent specific coloring
      self.drawPolygon2d(road_corridor.polygon, color="blue", alpha=.2)
      for lane_corridor in road_corridor.lane_corridors:
        self.drawLaneCorridor(lane_corridor)
