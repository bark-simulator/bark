# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import logging
from bark.core.viewer import Viewer
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.runtime.commons.parameters import ParameterServer
import math
from bark.core.world.evaluation.ltl import *
from bark.core.world.evaluation import *
from bark.core.models.behavior import *

logger = logging.getLogger()


class BufferedViewer(BaseViewer):
    def __init__(self, params=None, **kwargs):
        if(params is None):
            params = ParameterServer()
        BaseViewer.__init__(self)
      pass

    def reset():
      pass

    def get_aspect_ratio(self):
      pass

    def _get_draw_eval_agent_ids(self, world, eval_agent_ids=None):
      pass

    def _update_world_view_range(self, world, eval_agent_ids=None):
      pass

    def _update_world_dynamic_range(self, center):
      pass

    def drawPoint2d(self, point2d, color, alpha):
      pass

    def drawLine2d(self, line2d, color, alpha, line_style=None, zorder=2):
      pass

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None, zorder=10, hatch=''):
      pass

    def drawText(self, position, text, **kwargs):
      pass

    def getColor(self, color):
      pass

    def show(self, block=False):
      pass

    def clear(self):
      pass

    def drawAgents(self, world):
      pass

    def drawGoalDefinition(self, goal_definition, color, alpha, facecolor):
      pass

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
      # TODO: draw map
      
      # TODO: draw agents
      
      pass

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
      self.drawLine2d(lane.line, color, self.alpha_lane_boundaries,
                      dashed, zorder=1, linewidth=self.map_linewidth)

    def drawAgent(self, agent, color, alpha, facecolor, hatch=''):
      pass

    def drawLaneCorridor(self, lane_corridor, color=None):
      if color is None:
        color = "blue"
      self.drawPolygon2d(lane_corridor.polygon, color=color,
                         facecolor=color, alpha=.3, zorder=2)

    def Reset(self):
      pass

    def drawRoadCorridor(self, road_corridor, color=None):
      if color is None:
        color = "blue"
      self.drawPolygon2d(road_corridor.polygon, color,
                          facecolor=color, alpha=.2, zorder=2)

