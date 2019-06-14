

import numpy as np

from gym import spaces
from bark.models.behavior import BehaviorMotionPrimitives
from bark.models.dynamic import SingleTrackModel
from modules.runtime.commons.parameters import ParameterServer

class ActionWrapper:
    def action_to_behavior(self, world, actions):
        pass

    def reset(self, world, agents_to_act):
        pass

    @property
    def action_space(self):
        pass


class OpenAI(ActionWrapper):
    def reset(self, world, agents_to_act):
        if(len(agents_to_act) != 1):
            raise ValueError("Invalid number of actions given: {}".format(agents_to_act))


class MotionPrimitives(OpenAI):
    def __init__(self, params=ParameterServer()):
        self.params = params
        # todo: make parameterizable
        self.control_inputs = self.params["Runtime"]["RL"]["ActionWrapper"]["MotionPrimitives","Motion primitives available as discrete actions", \
                                                        [[0,0],[-20,0],[2,0],[-5,0],[0,60/2*3.14],[0,-120/2*3.14]]] # (acceleration, steering angle)
        self.behavior_model = None

    def reset(self, world, agents_to_act):
        super(MotionPrimitives, self).reset(world=world, agents_to_act=agents_to_act)
        self.behavior_model = BehaviorMotionPrimitives(SingleTrackModel(), self.params)
        for control_input in self.control_inputs:
            self.behavior_model.add_motion_primitive(np.array(control_input))
        ego_agent_id = agents_to_act[0]
        world.agents[ego_agent_id].behavior_model = self.behavior_model
        return world

    def action_to_behavior(self, world, action):
        if self.behavior_model:
            self.behavior_model.action_to_behavior(action)
        return world

    @property
    def action_space(self):
        return spaces.Discrete(len(self.control_inputs))

        

