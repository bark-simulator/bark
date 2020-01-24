# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import pygame as pg
import numpy as np
from bark.geometry import *
from bark.viewer import *
from bark.world.opendrive import *
from bark.models.dynamic import *
from modules.runtime.viewer.viewer import BaseViewer


class PygameViewer(BaseViewer):
    def __init__(self, params=None, **kwargs):
        super(PygameViewer, self).__init__(params=params, **kwargs)

        self.screen_dims = kwargs.pop("screen_dims", np.array([1024, 1024]))
        self.screen_width = self.screen_dims[0]
        self.screen_height = self.screen_dims[1]

        self.screen_map_ratio = max(self.screen_width/(np.diff(self.world_x_range)[0]),
                                    self.screen_height/(np.diff(self.world_y_range)[0]))
        self.camera_view_size = np.array([np.diff(self.dynamic_world_x_range)[0], np.diff(self.dynamic_world_y_range)[0]])

        self.map_surface = None
        self.map_size = None

        self.screen_surface = pg.Surface((self.screen_width, self.screen_height))

        pg.font.init()

        try:
            self.screen = pg.display.set_mode((self.screen_width, self.screen_height), pg.DOUBLEBUF)
            pg.display.set_caption("Bark")

            self.clear()
            self.show()
        except pg.error:
            self.screen = None
            print("No available video device")

    def drawMap(self, map):
        if self.map_surface == None:
            # find boundary to create fixed size pygame surface
            self.map_min_boundary = np.full(2, np.inf)
            self.map_max_boundary = np.full(2, -np.inf)
            lanes_np = []
            lanes_dashed = []

            for _, road in map.GetRoads().items():
                for lane_section in road.lane_sections:
                    for _, lane in lane_section.GetLanes().items():
                        lane_np = lane.line.ToArray()
                        lanes_np.append(lane_np)
                        lanes_dashed.append(lane.road_mark.type == XodrRoadMarkType.broken
                                            or lane.road_mark.type == XodrRoadMarkType.none)

                        self.map_min_boundary = np.minimum(self.map_min_boundary, np.amin(lane_np, axis=0))
                        self.map_max_boundary = np.maximum(self.map_max_boundary, np.amax(lane_np, axis=0))

            self.map_size = self.map_max_boundary-self.map_min_boundary
            # the size needed to be scaling larger for better visualization
            # as a pygame surface is stored as pixels
            self.map_surface = pg.Surface(tuple(self.map_size*self.screen_map_ratio))
            self.map_surface.fill((255, 255, 255))

            for lane_np, lane_dashed in zip(lanes_np, lanes_dashed):
                pg.draw.aalines(self.map_surface, self.getColor(self.color_lane_boundaries), False, self.mapToSurfaceCoordinates(lane_np), 3)

        camera_coordinate = self.mapToSurfaceCoordinates(np.array([self.dynamic_world_x_range[0], self.dynamic_world_y_range[1]]))
        camera_view_range = np.array([np.diff(self.dynamic_world_x_range)[0], np.diff(self.dynamic_world_y_range)[0]])*self.screen_map_ratio
        self.screen_surface.blit(self.map_surface, (0, 0), tuple(np.around(np.concatenate((camera_coordinate, camera_view_range)))))

    def drawPoint2d(self, point2d, color, alpha):
        pg.draw.circle(self.screen_surface, self.getColor(color), self.pointsToCameraCoordinate(point2d), 1, 0)

    def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False):
        # TODO: enable dashed line
        line2d = self.pointsToCameraCoordinate(line2d)
        pg.draw.lines(self.screen_surface, self.getColor(color), False, line2d, 3)

    def drawPolygon2d(self, polygon, color, alpha):
        points = self.pointsToCameraCoordinate(polygon)
        pg.draw.polygon(self.screen_surface, self.getColor(color), points)

    def drawTrajectory(self, trajectory, color):
        if len(trajectory) < 1:
            return
        point_list = []
        for state in trajectory:
            point_list.append([state[round(StateDefinition.X_POSITION)], state[round(StateDefinition.Y_POSITION)]])

        pg.draw.lines(self.screen_surface, self.getColor(color), False, self.pointsToCameraCoordinate(point_list), 5)

    def drawText(self, position, text, **kwargs):
        font = pg.font.get_default_font()
        fontsize = kwargs.pop("fontsize", 18)
        color = kwargs.pop("color", (0, 0, 0))
        background_color = kwargs.pop("background_color", (255, 255, 255))
        text_surface = pg.font.SysFont(font, fontsize).render(text, True, color, background_color)
        self.screen_surface.blit(text_surface, (position[0] * self.screen_width, position[1] * self.screen_height))

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

    def drawWorld(self, world, eval_agent_ids=None, show=True):
        self.clear()
        super(PygameViewer, self).drawWorld(world, eval_agent_ids)
        if show:
            self.show()

    def show(self, block=True):
        if self.screen is None:
            return
        self.screen.blit(self.screen_surface, (0, 0))
        pg.display.flip()
        pg.event.get()  # call necessary for visbility of pygame viewer on macos

    def clear(self):
        self.screen_surface.fill((255, 255, 255))

    """
        points: numpy array
        return: numpy array

        The origin of pygame surface is located at top left, increment downward
        therefore all the coordinates need to be transformed

        1. Mirror by y-axis
        2. Translate y-axis by the max y-coordinate in the map
        3. Translate x-axis by the max x-coordinate in the map
        4. Scaling by screen_map_ratio
    """

    def mapToSurfaceCoordinates(self, points):
        return (points * np.array([1, -1]) + np.array([-self.map_min_boundary[0], self.map_max_boundary[1]])) * self.screen_map_ratio

    def pointsToCameraCoordinate(self, points):
        if isinstance(points, list):
            points = np.array(points)
        elif not isinstance(points, np.ndarray):
            points = points.ToArray()

        return np.array([0, self.screen_height])+(points - np.array([self.dynamic_world_x_range[0], self.dynamic_world_y_range[0]]))*np.array([1, -1]) \
            / self.camera_view_size*self.screen_dims
