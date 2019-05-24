
from gym import spaces

class NNStateObserver:
    def observe(self, world, agents_to_observe):
        pass

    @property
    def observation_space(self):
        pass


class OpenAI(NNStateObserver):
    def observe(self, world, agents_to_observe):
        if(len(self.scenario.eval_agent_ids) != 1):
            raise ValueError("Invalid number of evaluation agents given: {}".format(agents_to_observe))


class StateConcatenation(OpenAI):
    def observe(self, world, agents_to_observe):
        super(StateConcatenation, self).observe(world=world, agents_to_observe=agents_to_observe)
        ego_observed_worlds = world.observe(agents_to_observe)[0]




