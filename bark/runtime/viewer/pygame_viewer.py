# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import pygame as pg
import numpy as np
from bark.core.geometry import *
from bark.core.viewer import *
from bark.core.world.opendrive import *
from bark.core.models.dynamic import *
from bark.runtime.viewer.viewer import BaseViewer


class PygameViewer(BaseViewer):
    def __init__(self, params=None, **kwargs):
        super(PygameViewer, self).__init__(params=params, **kwargs)

        self.screen_dims = kwargs.pop("screen_dims", np.array([600, 600]))
        self.screen_width = self.screen_dims[0]
        self.screen_height = self.screen_dims[1]
        self.screen_map_ratio = None
        self.source_dest = None
        self.screen_surface = pg.Surface(
            self.screen_dims, pg.DOUBLEBUF | pg.HWSURFACE)

        self.camera_view_size = None

        self.map_surf = None
        self.map_surf_size = None

        # NOTE: Optimizied way to support alpha value in pygame
        # https://stackoverflow.com/questions/6339057/draw-a-transparent-rectangle-in-pygame
        self.alpha_surf = dict()
        self.background_color = (255, 255, 255)

        pg.font.init()
        try:
            self.screen = pg.display.set_mode(
                self.screen_dims, pg.DOUBLEBUF | pg.HWSURFACE)
            pg.display.set_caption("Bark")
            self.clear()
        except pg.error:
            self.screen = None
            print("No available video device")

    def drawMap(self, map):
        """ Draw Opendrive map on a map surface and initialize camera surface properties according to the map size"""
        if self.map_surf is None:
            # Find boundary to create fixed size pygame surf
            self.map_min_bound = np.full(2, np.inf)
            self.map_max_bound = np.full(2, -np.inf)
            lanes_np = []
            lanes_dashed = []

            for _, road in map.GetRoads().items():
                for lane_section in road.lane_sections:
                    for _, lane in lane_section.GetLanes().items():
                        lane_np = lane.line.ToArray()
                        lanes_np.append(lane_np)
                        lanes_dashed.append(lane.road_mark.type == XodrRoadMarkType.broken
                                            or lane.road_mark.type == XodrRoadMarkType.none)

                        self.map_min_bound = np.minimum(
                            self.map_min_bound, np.amin(lane_np, axis=0))
                        self.map_max_bound = np.maximum(
                            self.map_max_bound, np.amax(lane_np, axis=0))

                self.camera_view_size = np.array([np.diff(self.dynamic_world_x_range)[
                    0], np.diff(self.dynamic_world_y_range)[0]])

                if self.use_world_bounds:
                    # Scale to the map size
                    self.screen_map_ratio = min(
                        self.screen_dims / self.camera_view_size)
                    self.map_surf_size = self.screen_dims
                else:
                    # Scale larger to have detailed visualization
                    self.screen_map_ratio = max([self.screen_width / (np.diff(self.world_x_range)[0]),
                                                 self.screen_height / (np.diff(self.world_y_range)[0])])
                    self.map_surf_size = (
                        self.map_max_bound - self.map_min_bound) * self.screen_map_ratio

            self.map_surf = pg.Surface(self.map_surf_size)
            self.map_surf.fill(self.background_color)

            for lane_np, lane_dashed in zip(lanes_np, lanes_dashed):
                if lane_dashed:
                    self.drawDashedLines(self.map_surf, self.getColor(
                        self.color_lane_boundaries), self.mapToSurfaceCoordinates(lane_np), 3)
                else:
                    pg.draw.lines(self.map_surf, self.getColor(
                        self.color_lane_boundaries), False, self.mapToSurfaceCoordinates(lane_np), 3)

        if self.use_world_bounds:
            camera_coordinate = np.array([0, 0])
            camera_view_range = self.screen_dims

            map_x_range = self.map_max_bound[0] - self.map_min_bound[0]
            map_y_range = self.map_max_bound[1] - self.map_min_bound[1]

            # Calculate source destination to blit map to the center of screen
            if map_x_range > map_y_range:
                self.source_dest = (
                    0, int(map_x_range - map_y_range) / 2 * self.screen_map_ratio)
            elif map_x_range < map_y_range:
                self.source_dest = (
                    int(map_y_range - map_x_range) / 2 * self.screen_map_ratio, 0)
            else:
                self.source_dest = (0, 0)

        else:
            # Project the coordinate of top left corner of dynamic world window
            camera_coordinate = self.mapToSurfaceCoordinates(
                np.array([self.dynamic_world_x_range[0], self.dynamic_world_y_range[1]]))
            camera_view_range = self.camera_view_size * self.screen_map_ratio
            self.source_dest = (0, 0)

        self.screen_surface.blit(self.map_surf, self.source_dest,
                                 np.around((camera_coordinate, camera_view_range)))

    def drawPoint2d(self, point2d, color, alpha=1.0):
        transformed_points = self.pointsToCameraCoordinate(point2d)
        s = self.createTransparentSurace(
            self.screen_dims, self.background_color, alpha)
        pg.draw.circle(s, self.getColor(color),
                       transformed_points, 1, 0)

    def drawLine2d(self, line2d, color="blue", alpha=1.0,
                   dashed=False, zorder=10, linewidth=1):
        transformed_lines = self.pointsToCameraCoordinate(line2d)
        s = self.createTransparentSurace(
            self.screen_dims, self.background_color, alpha)
        if dashed:
            self.drawDashedLines(
                s, self.getColor(color), transformed_lines, 3)

    def drawPolygon2d(self, polygon, color="blue", alpha=1.0, facecolor=None):
        transformed_points = self.pointsToCameraCoordinate(polygon)
        s = self.createTransparentSurace(
            self.screen_dims, self.background_color, alpha)
        pg.draw.polygon(s, self.getColor(color), transformed_points)

    def drawTrajectory(self, trajectory, color):
        if len(trajectory) < 1:
            return
        point_list = []
        for state in trajectory:
            point_list.append([state[round(StateDefinition.X_POSITION)],
                               state[round(StateDefinition.Y_POSITION)]])

        pg.draw.lines(self.screen_surface, self.getColor(color), False,
                      self.pointsToCameraCoordinate(point_list), 5)

    def drawText(self, position, text, **kwargs):
        font = pg.font.get_default_font()
        font_size = kwargs.pop("fontsize", 18)
        color = kwargs.pop("color", (0, 0, 0))
        background_color = kwargs.pop(
            "background_color", self.background_color)
        text_surf = pg.font.SysFont(font, font_size).render(
            text, True, color, background_color)
        self.screen_surface.blit(
            text_surf, (position[0] * self.screen_width, (1 - position[1]) * self.screen_height))

    def getColor(self, color):
        if isinstance(color, str):
            return {
                "white": pg.Color(255, 255, 255),
                "red": pg.Color(255, 0, 0),
                "blue": pg.Color(0, 0, 255),
                "magenta": pg.Color(100, 255, 100),
                "brown": pg.Color(150, 50, 50),
                "black": pg.Color(0, 0, 0)
            }.get(color, pg.Color(0, 0, 0))
        else:
            return pg.Color(int(color[0] * 255),
                            int(color[1] * 255),
                            int(color[2] * 255))

    def drawWorld(self, world, eval_agent_ids=None, show=True):
        super(PygameViewer, self).drawWorld(world, eval_agent_ids)

        if self.alpha_surf:
            for s in self.alpha_surf.values():
                self.screen_surface.blit(s, (0, 0))

        if show:
            self.show()

    def show(self, block=True):
        if self.screen is None:
            return

        self.screen.blit(self.screen_surface, (0, 0))

        pg.display.flip()
        pg.event.get()  # call necessary for visbility of pygame viewer on macos

    def clear(self):
        # Clear each surface, could occupy lot of memory if using many
        # different alpha values
        if self.alpha_surf:
            for s in self.alpha_surf.values():
                s.fill(self.background_color)

        self.screen.fill(self.background_color)
        self.screen_surface.fill(self.background_color)

    def getSizeOfColormap(self):
        # TODO
        return 1

    def getColorFromMap(self, double_color):
        # TODO
        return (1.0, 0, 0)

    def get_aspect_ratio(self):
        return 1

    """
        Steps to project coordinates:
        1. Mirror by y-axis
        2. Translate y-axis by the max y-coordinate in the map
        3. Translate x-axis by the max x-coordinate in the map
        4. Scaling by screen_map_ratio
    """

    def mapToSurfaceCoordinates(self, points):
        """ Project Opendrive map coordinates into pygame surface coordinates"""
        return (points - np.array([self.map_min_bound[0], self.map_max_bound[1]])
                ) * np.array([1, -1]) * self.screen_map_ratio

    def pointsToCameraCoordinate(self, points):
        """ Project Opendrive map coordinates into camera surface coordinates"""
        if isinstance(points, list):
            points = np.array(points)
        elif not isinstance(points, np.ndarray):
            points = points.ToArray()

        if self.use_world_bounds:
            return np.array([0, self.screen_height]) + (points - np.array([self.dynamic_world_x_range[0], self.dynamic_world_y_range[0]])) * np.array([1, -1]) \
                * self.screen_map_ratio
        else:
            return np.array([0, self.screen_height]) + (points - np.array([self.dynamic_world_x_range[0], self.dynamic_world_y_range[0]])) * np.array([1, -1]) \
                / self.camera_view_size * self.screen_dims

    def createTransparentSurace(self, dims, background_color, alpha):
        alpha = round(float(alpha), 2)
        if alpha not in self.alpha_surf:
            s = pg.Surface(dims, pg.DOUBLEBUF | pg.HWSURFACE)
            s.fill(background_color)
            s.set_colorkey(background_color)
            s.set_alpha(int(alpha * 255))
            self.alpha_surf[alpha] = s
            return s
        else:
            return self.alpha_surf[alpha]

    def drawDashedLines(self, surf, color, points, width, dashed_length=5):
        for i in range(len(points) - 1):
            origin = points[i]
            target = points[i + 1]
            diff = target - origin
            length = np.linalg.norm(diff)
            slope = diff / length

            for j in np.arange(0, length / dashed_length, 2):
                start = origin + slope * j * dashed_length
                end = start + slope * dashed_length
                pg.draw.line(surf, color, start, end, width)
