

import numpy as np

from modules.runtime.ml.spaces import Discrete, Continuous
from bark.models.behavior import BehaviorMotionPrimitives, DynamicBehaviorModel
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
                                                        [[0,0], [10,0],[-2,0],[-50,0]]] # (acceleration, steering angle)
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
        return Discrete(len(self.control_inputs))


class DynamicModel(OpenAI):
    def __init__(self,
                 params=ParameterServer(),
                 dynamic_model=SingleTrackModel()):
        self.params_ = params
        self.control_inputs_ = self.params_["Runtime"]["RL"]["ActionWrapper"]["MotionPrimitives","Motion primitives available as discrete actions", \
                                                        [[0,0], [10,0],[-2,0],[-50,0]]] # (acceleration, steering angle)
        self.dynamic_model_ = dynamic_model
        self.behavior_model_ = DynamicBehaviorModel(dynamic_model,
                                                    self.params_)

    def reset(self, world, agents_to_act):
        super(DynamicModel, self).reset(world=world,
                                        agents_to_act=agents_to_act)
        self.behavior_model_ = DynamicBehaviorModel(self.dynamic_model_,
                                                    self.params_)
        ego_agent_id = agents_to_act[0]
        world.agents[ego_agent_id].behavior_model = self.behavior_model_
        return world

    def action_to_behavior(self, world, action):
        if self.behavior_model_:
            self.behavior_model_.set_action(action)
        return world

    @property
    def action_space(self):
        return Continuous(2)

        

