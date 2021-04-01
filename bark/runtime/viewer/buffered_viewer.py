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
    self._vparams = self._params["Visualization"]
    # map color config
    self._dashed_sc = self._vparams[
      "DashedStrokeColor", "", [128, 128, 128, 64]] 
    self._dashed_sw = self._vparams["DashedStrokeWidth", "",  0.1]
    self._solid_sc = self._vparams[
      "SolidStrokeColor", "", [12, 44, 132, 128]]
    self._solid_sw = self._vparams["SolidStrokeWidth", "", 0.2]
    # agent color config
    self._ego_sc = self._vparams[
      "EgoStrokeColor", "", [34, 94, 168, 255]] 
    self._ego_fc = self._vparams[
      "EgoFaceColor", "", [34, 94, 168, 128]] 
    self._other_sc = self._vparams[
      "OtherStrokeColor", "", [128, 128, 128, 255]] 
    self._other_fc = self._vparams[
      "OtherFaceColor", "", [128, 128, 128, 128]] 
    self._agent_height = self._vparams[
      "AgentHeight", "", 2.] 
    
  def DrawAgents(self, world, eval_agent_id):
    ego_agent = world.agents[eval_agent_id]
    for agent in world.agents.values():
      agent_poly = agent.GetPolygonFromState(agent.state)
      agent_primitive = RenderPrimitive(agent_poly)
      agent_primitive.Add("agent_id", str(agent.id))
      agent_primitive.Add("height", self._agent_height)
      if agent == ego_agent:
        agent_primitive.Add("stroke_color", self._ego_sc)
        agent_primitive.Add("fill_color", self._ego_fc)
        world.renderer.Add("EGO_AGENT", agent_primitive)    
      else:
        agent_primitive.Add("stroke_color", self._other_sc)
        agent_primitive.Add("fill_color", self._other_fc)
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
    lane_primitive.Add("stroke_color", self._solid_sc)
    lane_primitive.Add("stroke_width", self._solid_sw)
    if lane.road_mark.type == XodrRoadMarkType.broken or \
      lane.road_mark.type == XodrRoadMarkType.none:
      lane_primitive.Add("stroke_color", self._dashed_sc)
      lane_primitive.Add("stroke_width", self._dashed_sw)
    world.renderer.Add("MAP_LINE", lane_primitive)

  # NOTE: legacy functions
  def clear(self):
    pass
  def Reset(self):
    pass
