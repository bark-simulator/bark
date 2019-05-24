

from gym import spaces
from bark.models.behavior import BehaviorDeltaState

class ActionWrapper:
    def action_to_behavior(self, world, agents_to_act, action):
        pass

    @property
    def action_space(self):
        pass


class OpenAI(ActionWrapper):
    def action_to_behavior(self, world, agents_to_act, action):
        if(len(self.scenario.eval_agent_ids) != 1):
            raise ValueError("Invalid number of actions given: {}".format(agents_to_act))


class MotionPrimitives(OpenAI):
    def __init__(self):
        # todo: make parameterizable
        self.motion_primitives = [(2,0),(-5,0),(2,30/2*3.14)] # (acceleration, steering angle)

    def action_to_behavior(self, world, agents_to_act, action):
        super(MotionPrimitives, self).action_to_behavior(world=world, agents_to_act=agents_to_act, action=action)
        ego_agent_id = agents_to_act[0]
        # todo implement motion primitive model
        world.agents[ego_agent_id].behavior_model = BehaviorDeltaState(action)
        return world

    @property
    def action_space(self):
        return space.Discrete(len(self.motion_primitives))

        

