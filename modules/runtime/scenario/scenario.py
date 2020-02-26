# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.world.agent import Agent
from bark.world import World
from bark.world.map import MapInterface
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser
import copy


class Scenario:
  def __init__(self,
               agent_list=None,
               eval_agent_ids=None,
               map_file_name=None,
               json_params=None,
               map_interface=None):
    self._agent_list = agent_list or []
    self._eval_agent_ids = eval_agent_ids or []
    self._map_file_name = map_file_name
    self._json_params = json_params
    self._map_interface = map_interface

  @property
  def map_file_name(self):
    return self._map_file_name
  
  @property
  def json_params(self):
    return self._json_params

  @property
  def eval_agent_ids(self):
    return self._eval_agent_ids

  def get_world_state(self):
    """get initial world state of scenario to start simulation from here
    
    Returns:
        [bark.world.World]
    """
    return self._build_world_state()

  def copy(self):
    return Scenario(agent_list=copy.deepcopy(self._agent_list),
                    eval_agent_ids=self._eval_agent_ids.copy(),
                    map_file_name=self._map_file_name,
                    json_params=self._json_params.copy(),
                    map_interface=self._map_interface)

  def _build_world_state(self):
    param_server = ParameterServer(json=self._json_params)
    world = World(param_server)
    if self._map_interface is None:
      world = self.setup_map(world, self._map_file_name)
    else:
      world.SetMap(self._map_interface)
    for agent in self._agent_list:
      agent.GenerateRoadCorridor(self._map_interface)
      world.AddAgent(agent)
    return world

  def __getstate__(self):
    odict = self.__dict__.copy()
    del odict['_map_interface']
    return odict

  def __setstate__(self, sdict):
    sdict['_map_interface'] = None
    self.__dict__.update(sdict)

  def setup_map(self, world, _map_file_name):
    if not _map_file_name:
        return world
    xodr_parser = XodrParser(_map_file_name )
    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    self._map_interface = map_interface
    world.SetMap(map_interface)
    return world





