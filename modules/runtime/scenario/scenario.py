# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

class Scenario:
    def __init__(self, world_state=None, description=None):
        self.world_state = world_state
        self.description = description
        self.goal_criteria = []

    def set_world_state(self, world_state):
        """ sets initial world state of this scenario
        
        Arguments:
            world_state {[bark.world.World]} -- [pass a full world instance giving the actual world state]
        """

        self.world_state = world_state

    def get_world_state(self):
        """get initial world state of scenario to start simulation from here
        
        Returns:
            [bark.world.World]
        """
        return self.world_state

    def set_description(description):
        """set scenario description
        
        Arguments:
            description {[dict]} -- [pass a dictionary with type and other info of this scenario....to be defined what it must contain]
        """

    def get_description(self):
        return self.description

    def add_goal_criterion(self, goal_criterion):
        """add a goal specification to this scenario, e.g. drive into a certain goal region, or maximum amount of time 10s, multiple criteria can be added subsequently
        
        Arguments:
            goal_criterion {[todo]} -- [todo]
        """

        self.goal_criteria.append(goal_criterion)


    def copy(self):
        return Scenario(world_state=self.world_state.copy(), description=self.description.copy())





