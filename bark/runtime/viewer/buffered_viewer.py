# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import math
import numpy as np
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.world.renderer import *
from bark.runtime.commons.parameters import ParameterServer


class BufferedViewer:
  def __init__(self, params=None, **kwargs):
    self._params = params or ParameterServer()

  def DrawAgents(self, world, eval_agent_id):
    ego_agent = world.agents[eval_agent_id]
    for agent in world.agents.values():
      agent_poly = agent.GetPolygonFromState(agent.state)
      # TODO: add styles
      agent_primitive = RenderPrimitive(agent_poly)
      agent_primitive.conf["agent_id"] = str(agent.id)
      if agent == ego_agent:
        agent_primitive.conf["line_color"] = "blue"
        agent_primitive.conf["face_color"] = "green"
        world.renderer.Add("EGO_AGENT", agent_primitive)    
      else:
        agent_primitive.conf["line_color"] = "yellow"
        agent_primitive.conf["face_color"] = "red"
        world.renderer.Add("OTHER_AGENT", agent_primitive)
      
  def drawWorld(self, world, eval_agent_ids=None, scenario_idx=None):
    self.DrawMap(world)
    self.DrawAgents(world, eval_agent_ids[0])
    # add ego state
    ego_agent = world.agents[eval_agent_ids[0]]
    ego_agent_state = ego_agent.state
    ego_vel_primitive = RenderPrimitive(ego_agent_state)
    world.renderer.Add("EGO_AGENT_STATE", ego_vel_primitive)
    # TODO: e.g., set map origin

  def DrawMap(self, world):
    map = world.map.GetOpenDriveMap()
    for road in map.GetRoads().values():
      for lane_section in road.lane_sections:
        for lane in lane_section.GetLanes().values():
          self.DrawLane(lane, world)
      
  def DrawLane(self, lane, world):
    lane_primitive = RenderPrimitive(lane.line)
    # TODO: add styles
    lane_primitive.conf["line_color"] = "blue"
    if lane.road_mark.type == XodrRoadMarkType.broken or \
      lane.road_mark.type == XodrRoadMarkType.none:
      lane_primitive.conf["line_color"] = "gray"
    world.renderer.Add("MAP_LINE", lane_primitive)

  def clear(self):
    pass
  
  def Reset(self):
    pass
