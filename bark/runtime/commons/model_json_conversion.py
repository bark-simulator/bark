# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.core.world.agent import *
from bark.core.models.behavior import *
from bark.core.world import *
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.runtime.commons import ParameterServer
import inspect
import re
import sys
import numpy as np


class ModelJsonConversion:

  def agent_from_json(self, agent_json, param_server):
    bark_agent = Agent(
      np.array(agent_json["state"]), 
      self.convert_model(agent_json["behavior_model"], param_server), 
      self.convert_model(agent_json["dynamic_model"], param_server),
      self.convert_model(agent_json["execution_model"], param_server), 
      Polygon2d(agent_json["shape"]["center_pose"],
      np.array(agent_json["shape"]["polygon_points"])),
      param_server.AddChild("agent"),
      agent_json["goal_definition"],
      agent_json["map_interface"])
    return bark_agent

  def agent_to_json(self, agent):
    agent_json = {}
    agent_json["behavior_model"] = self.convert_model(agent.behavior_model)
    agent_json["dynamic_model"] = self.convert_model(agent.dynamic_model)
    agent_json["execution_model"] = self.convert_model(agent.execution_model)
    agent_json["id"] = agent.id
    agent_json["state"] = agent.state.tolist()
    agent_json["shape"] = {}
    agent_json["shape"]["polygon_points"] = agent.shape.ToArray().tolist()
    agent_json["shape"]["center_pose"] = agent.shape.center.tolist()
    agent_json["followed_trajectory"] = agent.followed_trajectory.tolist()
    agent_json["planned_trajectory"] = agent.planned_trajectory.tolist()
    return agent_json

  def convert_model(self, model, params=None, dynamic_model=None):
    if isinstance(model,str):
      try:
        if params is None:
          return eval("{}()".format(model))
        else:
          try:
            return eval("{}(params)".format(model))    
          except:
            return eval("{}(dynamic_model, params)".format(model))                 
      except NameError:
        raise NameError("Unkown model type: {}".format(input))
    else:
        return str(model).split(".")[-1]

  def extract_models(self, module, filter_regexp):
    all_module_members = inspect.getmembers(sys.modules[module])
    model_list = []
    for module_member in all_module_members:
      member_name = module_member[0]
      m = re.match(filter_regexp,member_name)
      if m:
        model_list.append(m.group())
    return model_list
