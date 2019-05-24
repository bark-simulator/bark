



from bark.runtime.runtime import Runtime
from gym.spaces import Box


class RuntimeRL(Runtime):
    def __init__(self, action_wrapper, nn_observer, reward_observer, step_time, viewer, scenario_generator=None):
        super().__init__(step_time=step_time, viewer=viewer, scenario_generator=scenario_generator)
        self.action_wrapper = action_wrapper
        self.nn_observer = nn_observer
        self.reward_observer = reward_observer

    def reset(self, scenario=None)
        super().reset(scenario=scenario)
        return self.nn_observer.observe(world=self.world, agents_to_observe=self.scenario.eval_agents_ids)

    def step(self, action=None)
        self.world = self.action_wrapper.action_to_behavior(world=self.world, agents_to_act=self.scenario.eval_agents_ids, action=action)
        self.world.step()
        return self.nn_observer.observe(world=self.world, agents_to_observe=self.scenario.eval_agents_ids)

    def action_space(self):
        return self.action_wrapper.action_space

    def observation_space(self):
        return self.nn_observer.observation_space

    def get_nstate_reward_action_tuple(self, world, controlled_agents ):
        next_state, done, success = self.nn_observer.observe(world=self.world, agents_to_observe=controlled_agents)
        reward = self.reward_observer.get_reward(world=world, agents_to_observe=controlled_agents)
        return next_state, reward, done, success


