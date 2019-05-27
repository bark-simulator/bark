
from gym import spaces

class NNStateObserver:
    def observe(self, world, agents_to_observe):
        pass

    @property
    def observation_space(self):
        pass


class OpenAI(NNStateObserver):
    def observe(self, world, agents_to_observe):
        if(len(agents_to_observe) != 1):
            raise ValueError("Invalid number of evaluation agents given: {}".format(agents_to_observe))


class StateConcatenation(OpenAI):
    def observe(self, world, agents_to_observe):
        super(StateConcatenation, self).observe(world=world, agents_to_observe=agents_to_observe)
        ego_observed_world = world.observe(agents_to_observe)[0]
        
        # create one 
        num_other_agents = len(ego_observed_world.other_agents)

        # fill vector, use relative state difference for ego
        ego_state = ego_observed_world.ego_agent.state
        return None, None, None




