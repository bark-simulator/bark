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


class BufferedViewer:
  def __init__(self, params=None, **kwargs):
    self._params = params or ParameterServer()
    self._renderer = Renderer()

  def DrawMap(self, world):
    for road in map.GetRoads().values():
      for lane_section in road.lane_sections:
        for lane in lane_section.GetLanes().values():
          self.DrawLane(lane)
      
  def DrawLane(self, lane):
    line_style = "solid"
    if lane.road_mark.type == XodrRoadMarkType.broken or \
      lane.road_mark.type == XodrRoadMarkType.none:
      line_style = dashed
    # TODO:
    # self._renderer.Add()

  def DrawAgents(self, world, eval_agent_ids):
    ego_id = eval_agent_ids[0]
    ego_agent = world.agents[ego_id]
    for agent in world.agents.values():
      pass      

  def drawWorld(self, world, eval_agent_ids=None):
    # TODO: draw map
    self.DrawMap(world)
    
    # TODO: draw agents
    
  def clear(self):
    self._renderer.Clear()
  
  def Reset(self):
    self._renderer.Clear()
