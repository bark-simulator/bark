
from gym import spaces
import numpy as np
from bark.models.dynamic import StateDefinition
from modules.runtime.commons.parameters import ParameterServer
import math
import operator

class NNStateObserver:
    def __init__(self):
        self.world_x_range = None
        self.world_y_range = None

    def observe(self, world, agents_to_observe):
        pass

    def reset(self, world, agents_to_observe):
        bb = world.bounding_box
        self.world_x_range = [bb[0].x(), bb[1].x()]
        self.world_y_range = [bb[0].y(), bb[1].y()]
        return world

    @property
    def observation_space(self):
        pass


class OpenAI(NNStateObserver):
    def observe(self, world, agents_to_observe):
        if(len(agents_to_observe) != 1):
            raise ValueError("Invalid number of evaluation agents given: {}".format(agents_to_observe))


class StateConcatenation(OpenAI):
    def __init__(self, params=ParameterServer()):
        # todo make parameterizable
        self.nn_state_dimensions = [int(StateDefinition.X_POSITION),
                                          int(StateDefinition.Y_POSITION),
                                          int(StateDefinition.THETA_POSITION),
                                          int(StateDefinition.VEL_POSITION)]

        self.params = params
        self.velocity_range = self.params["Runtime"]["RL"]["StateConcatenation"]["VelocityRange","Gives boundaries for min and max velocity for normalization", [0, 100]]
        self.theta_range = self.params["Runtime"]["RL"]["StateConcatenation"]["ThetaRange","Gives boundaries for min and max theta for normalization", [0, 2*math.pi]]
        self.normalize = self.params["Runtime"]["RL"]["StateConcatenation"]["Normalize","Should normalization be performed", True]
        self.max_num_other_agents = self.params["Runtime"]["RL"]["StateConcatenation"]["MaxOtherAgents","The concatenation state size is ego agent plus max num other agents", 4]
        self.max_distance_other_agents = self.params["Runtime"]["RL"]["StateConcatenation"]["MaxOtherDistance","Agents farer than this value are not observed; \
                                                                                     if not max other agents are seen, remaining concatenation state is set to zero", 30]

    def observe(self, world, agents_to_observe):
        super(StateConcatenation, self).observe(world=world, agents_to_observe=agents_to_observe)
        ego_observed_world = world.observe(agents_to_observe)[0]
        
        num_other_agents = len(ego_observed_world.other_agents)
        ego_state = ego_observed_world.ego_agent.state

        # calculate nearest agent distances
        nearest_distances = {}
        for agent_id, agent in ego_observed_world.other_agents.items():
            if agent_id == agents_to_observe[0]:
                continue
            dist = (ego_state[int(StateDefinition.X_POSITION)]-agent.state[int(StateDefinition.X_POSITION)])**2 + \
                (ego_state[int(StateDefinition.Y_POSITION)]-agent.state[int(StateDefinition.Y_POSITION)])**2
            nearest_distances[dist] = agent_id

        # preallocate numpyarray and add ego state
        concatenated_state = np.zeros((self._len_ego_state+self.max_num_other_agents*self._len_relative_agent_state,1))
        concatenated_state[0:self._len_ego_state] = np.reshape(self._reduce_to_ego_nn_state( \
                                            self._norm(ego_state)) , (self._len_ego_state ,1))
        
        # add max number of agents to state concatenation vector
        concat_pos = self._len_relative_agent_state
        nearest_distances = sorted(nearest_distances.items(), key=operator.itemgetter(0))
        for agent_idx in range(0, self.max_num_other_agents):
            if agent_idx<len(nearest_distances) and nearest_distances[agent_idx][0] <= self.max_distance_other_agents**2:
                agent_id = nearest_distances[agent_idx][1]
                agent = ego_observed_world.other_agents[agent_id]
                agent_rel_state = self._reduce_to_other_nn_state( \
                                self._calculate_relative_agent_state(ego_state, self._norm(agent.state)))
                concatenated_state[concat_pos:concat_pos + self._len_relative_agent_state] = np.reshape(agent_rel_state, (self._len_relative_agent_state, 1))
            else:
                concatenated_state[concat_pos:concat_pos + self._len_relative_agent_state] = np.zeros((self._len_relative_agent_state, 1))
            concat_pos += self._len_relative_agent_state
        
        return concatenated_state

    def observation_space(self):
        return spaces.Box(low=np.zeros((self._len_ego_state+self.max_num_other_agents*self._len_relative_agent_state,1)), \
                            high = np.ones((self._len_ego_state+self.max_num_other_agents*self._len_relative_agent_state,1)) )

    def _norm(self, agent_state):
        if not self.normalize:
            return agent_state
        agent_state[int(StateDefinition.X_POSITION)] = \
            self._norm_to_range(agent_state[int(StateDefinition.X_POSITION)], self.world_x_range)
        agent_state[int(StateDefinition.Y_POSITION)] = \
            self._norm_to_range(agent_state[int(StateDefinition.Y_POSITION)], self.world_y_range)
        agent_state[int(StateDefinition.THETA_POSITION)] = \
            self._norm_to_range(agent_state[int(StateDefinition.THETA_POSITION)], self.theta_range)
        agent_state[int(StateDefinition.VEL_POSITION)] = \
            self._norm_to_range(agent_state[int(StateDefinition.VEL_POSITION)], self.velocity_range)
        return agent_state

    def _reduce_to_ego_nn_state(self, agent_state):
        return agent_state[self.nn_state_dimensions]

    def _reduce_to_other_nn_state(self, agent_state):
        return agent_state[self.nn_state_dimensions]

    def _norm_to_range(self, value, range):
        return (value - range[0])/(range[1]-range[0])

    def _calculate_relative_agent_state(self, ego_agent_state, agent_state):
        return agent_state

    @property
    def _len_relative_agent_state(self):
        return len(self.nn_state_dimensions)

    @property
    def _len_ego_state(self):
        return len(self.nn_state_dimensions)




