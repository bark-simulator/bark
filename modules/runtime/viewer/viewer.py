# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
from bark.viewer import Viewer
from bark.geometry import *
from bark.models.dynamic import *


class BaseViewer(Viewer):
    def __init__(self, params=None, **kwargs):
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
        # map
        self.color_lane_boundaries = params["Visualization"]["Map"]["Lanes"]["Boundaries"]["Color", "Color of agents except ego vehicle", (0.7,0.7,0.7)]
        self.alpha_lane_boundaries = params["Visualization"]["Map"]["Lanes"]["Boundaries"]["Alpha", "Color of agents except ego vehicle", 1.0]
        self.plane_color = params["Visualization"]["Map"]["Plane"]["Color", "Color of the background plane", (1, 1, 1, 1)]
        self.plane_alpha = params["Visualization"]["Map"]["Plane"]["Alpha", "Alpha of the background plane", 1.0]


        self.parameters = params

        self.world_x_range = kwargs.pop("x_range", [-40, 40])
        self.world_y_range = kwargs.pop("y_range", [-40, 40])
        self.use_world_bounds = kwargs.pop("use_world_bounds", False)
        self.follow_agent_id = kwargs.pop("follow_agent_id", None)

        self.dynamic_world_x_range = self.world_x_range.copy()
        self.dynamic_world_y_range = self.world_y_range.copy()

    def _update_world_view_range(self, world, eval_agent_ids=None):
        if self.follow_agent_id is not None:
            if isinstance(self.follow_agent_id, bool) and \
                     eval_agent_ids is not None and \
                     len(eval_agent_ids) == 1:
                follow_agent =world.agents[eval_agent_ids[0]]
            else:
                follow_agent = world.agents[self.follow_agent_id]
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

    def drawLine2d(self, line2d, color, alpha):
        pass

    def drawPolygon2d(self, polygon, color, alpha):
        pass

    def drawTrajectory(self, trajectory, color):
        pass

    def drawObstacle(self, obstacle):
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

    def drawWorld(self, world, eval_agent_ids=None):
        self._update_world_view_range(world, eval_agent_ids)
        self.drawMap(world.map.get_open_drive_map())

        # draw agents
        for _, agent in world.agents.items():
            if eval_agent_ids and agent.id in eval_agent_ids:
                color = self.color_eval_agents
            else:
                color = self.color_other_agents
            self.drawAgent(agent, color)

            if self.draw_eval_goals:
                self.drawPolygon2d(agent.goal_definition.goal_shape, self.eval_goal_color, alpha=0.9)


    def drawMap(self, map):
        # draw the boundary of each lane
        for _, road in map.get_roads().items():
            for lane_section in road.lane_sections:
                for _, lane in lane_section.get_lanes().items():
                    self.drawLine2d(lane.line, self.color_lane_boundaries, self.alpha_lane_boundaries)


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

    def drawDrivingCorridor(self, corridor, color):
        self.drawLine2d(corridor.center, color, 1)
        self.drawLine2d(corridor.inner, color, 1)
        self.drawLine2d(corridor.outer, color, 1)

    def drawRoute(self, agent):
        # TODO(@hart): visualize the global as well as the local driving corridor
        self.drawDrivingCorridor(agent.local_map.get_driving_corridor(), self.route_color)
        self.drawDrivingCorridor(agent.local_map.get_horizon_driving_corridor(), (0.8, 0.72, 0.2))




