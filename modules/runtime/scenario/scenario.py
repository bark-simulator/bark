# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import copy


class Scenario:
    def __init__(self, agent_list=None, eval_agent_ids=None, map_file_name=None, json_params=None):
        self.agent_list = agent_list or []
        self.eval_agent_ids = eval_agent_ids or []
        self.map_file_name = map_file_name
        self.json_params = json_params

    def get_world_state(self):
        """get initial world state of scenario to start simulation from here
        
        Returns:
            [bark.world.World]
        """
        return self._build_world_state()

    def copy(self):
        return Scenario(agent_list=copy.deepcopy(self.agent_list), eval_agent_ids=self.eval_agent_ids.copy(),
                                 map_file_name=self.map_file_name, json_params=self.json_params.copy())

    def _build_world_state(self):
        param_server = ParameterServer(json=self.json_params)
        param_server
        world = World(param_server)
        world = self.setup_map(world,self.map_file_name)

        for agent in self.agent_list:
            world.add_agent(agent)
        return world

    def setup_map(self, world, map_file_name):
        xodr_parser = XodrParser(map_file_name )
        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)
        return world





