# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import pygame as pg
import numpy as np
from bark.geometry import *
from bark.viewer import *
from bark.models.dynamic import *
from modules.runtime.viewer.viewer import BaseViewer


class PygameViewer(BaseViewer):
    def __init__(self, params=None, **kwargs):
        super(PygameViewer, self).__init__(params=params)
        self.world_x_range = kwargs.pop("x_range", [-40, 40])
        self.world_y_range = kwargs.pop("y_range", [-40, 40])
        self.use_world_bounds = kwargs.pop("use_world_bounds", False)
        self.follow_agent_id = kwargs.pop("follow_agent_id", None)
        self.screen_dims = kwargs.pop("screen_dims", [1024, 1024])
        self.screen_width = self.screen_dims[0]
        self.screen_height = self.screen_dims[1]

        self.dynamic_world_x_range = self.world_x_range.copy()
        self.dynamic_world_y_range = self.world_y_range.copy()

        try:
            self.screen = pg.display.set_mode((self.screen_width,
                                               self.screen_height))
            pg.display.set_caption("Bark")

            self.clear()
            self.show()
        except pg.error:
            self.screen = None
            print("No available video device")

    def _update_world_view_range(self, world):
        if self.follow_agent_id is not None:
            follow_agent = world.agents[self.follow_agent_id]
            state = follow_agent.followed_trajectory[-1,:]
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
        if self.screen is None:
            return
        pg.draw.circle(self.screen, self.getColor(color),
                       self.pointToPG(point2d), 1, 0)

    def drawLine2d(self, line2d, color='blue', alpha=1.0):
        if self.screen is None:
            return
        line2d_np = line2d.toArray()
        line_transformed_np = np.apply_along_axis(self.pointToPG, axis=1, arr=line2d_np)
        for line_slice in np.ma.clump_unmasked(np.ma.masked_invalid(line_transformed_np)):
            line_np = line_transformed_np[line_slice]
            if len(line_np.tolist()) < 2:
                continue
            pg.draw.lines(
                self.screen, self.getColor(color), False,
                line_np.tolist(), 3)

    def drawPolygon2d(self, polygon, color, alpha):
        if self.screen is None:
            return
        points = polygon.toArray()
        points_transformed_np = np.apply_along_axis(self.pointToPG, axis=1, arr=points)
        points_transformed_np = points_transformed_np[~np.isnan(points_transformed_np).any(axis=1)]
        if len(points_transformed_np.tolist()) < 2:
            return

        pg.draw.polygon(self.screen, self.getColor(color), points_transformed_np.tolist())

    def drawTrajectory(self, trajectory, color):
        if self.screen is None:
            return
        if len(trajectory) < 1:
            return
        point_list = []
        for state in trajectory:
            point_list.append([
                self.x_worldToPyGameCoordinate(state[int(
                    StateDefinition.X_POSITION)]),
                self.y_worldToPyGameCoordinate(state[int(
                    StateDefinition.Y_POSITION)])
            ])
        pg.draw.lines(self.screen, self.getColor(color), False, point_list, 5)

    def getColor(self, color):
        if isinstance(color, Viewer.Color):
            return {
                Viewer.Color.White: (255, 255, 255),
                Viewer.Color.Red: (255, 0, 0),
                Viewer.Color.Blue: (0, 0, 255),
                Viewer.Color.Magenta: (100, 255, 100),
                Viewer.Color.Brown: (150, 50, 50),
                Viewer.Color.Black: (0, 0, 0)
            }.get(color, (0, 0, 0))
        else:
            return (color[0] * 255, color[1] * 255, color[2] * 255)

    def drawWorld(self, world, eval_agent_ids=None):
        self.clear()
        self._update_world_view_range(world)
        super(PygameViewer, self).drawWorld(world, eval_agent_ids)
        self.show()

    def show(self, block=True):
        if self.screen is None:
            return
        pg.display.update()

    def clear(self):
        if self.screen is None:
            return
        self.screen.fill((255, 255, 255))

    def _map_world_coordinate_to_screen_coordinate(self, world_coordinate, world_coordinate_range, screen_range):
        delta_world = world_coordinate_range[1] - world_coordinate_range[0]
        delta_screen = screen_range[1] - screen_range[0]

        return round(
            ( world_coordinate - world_coordinate_range[0] ) / delta_world * delta_screen + screen_range[0]
        )


    def x_worldToPyGameCoordinate(self, x_world):
        return self._map_world_coordinate_to_screen_coordinate(x_world, self.dynamic_world_x_range,(0,self.screen_width))

    def y_worldToPyGameCoordinate(self, y_world):
        return self.screen_height - self._map_world_coordinate_to_screen_coordinate(y_world, self.dynamic_world_y_range,(0,self.screen_height))

    def pointToPG(self, point):
        if isinstance(point, Point2d):
            x = self.x_worldToPyGameCoordinate(point.x())
            y = self.y_worldToPyGameCoordinate(point.y())

        elif isinstance(point, np.ndarray):
            x = self.x_worldToPyGameCoordinate(point[0])
            y = self.y_worldToPyGameCoordinate(point[1])

        if x is not np.nan and y is not np.nan:
            return (x,y)
        else:
            return (np.nan, np.nan)
