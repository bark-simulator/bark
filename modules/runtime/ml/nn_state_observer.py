
from gym import spaces
import numpy as np
from bark.models.dynamic import StateDefinition

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
    def __init__(self):
        # todo make parameterizable
        self.observed_state_dimensions = [int(StateDefinition.X_POSITION),
                                          int(StateDefinition.Y_POSITION),
                                          int(StateDefinition.THETA_POSITION),
                                          int(StateDefinition.VEL_POSITION)]

    def observe(self, world, agents_to_observe):
        super(StateConcatenation, self).observe(world=world, agents_to_observe=agents_to_observe)
        ego_observed_world = world.observe(agents_to_observe)[0]
        
        # create one 
        num_other_agents = len(ego_observed_world.other_agents)
        ego_state = ego_observed_world.ego_agent.state
        concatenated_state = np.zeros((ego_state.size+num_other_agents*self._len_relative_agent_state,1))
        # fill vector, use relative state difference for ego
        
        concatenated_state[0:ego_state.size] = np.reshape(ego_state, (ego_state.size ,1))

        concat_pos = self._len_relative_agent_state
        for agent_id, agent in ego_observed_world.other_agents.items():
            agent_rel_state = self._calculate_relative_agent_state(ego_state, agent.state)
            concatenated_state[concat_pos:concat_pos + self._len_relative_agent_state] = np.reshape(agent_rel_state, (self._len_relative_agent_state, 1))
            concat_pos += self._len_relative_agent_state
        
        return concatenated_state, None, None

    def observation_space(self):
        pass

    def _calculate_relative_agent_state(self, ego_agent_state, agent_state):
        return agent_state[np.array(self.observed_state_dimensions)]

    @property
    def _len_relative_agent_state(self):
        return len(self.observed_state_dimensions)




