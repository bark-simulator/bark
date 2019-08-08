



from modules.runtime.runtime import Runtime
from gym.spaces import Box


class RuntimeRL(Runtime):
    def __init__(self, action_wrapper, nn_observer, evaluator, step_time, viewer, scenario_generator=None):
        super().__init__(step_time=step_time, viewer=viewer, scenario_generator=scenario_generator)
        self.action_wrapper = action_wrapper
        self.nn_observer = nn_observer
        self.evaluator = evaluator

    def reset(self, scenario=None):
        super().reset(scenario=scenario)
        self.world = self.evaluator.reset(self.world, self.scenario.eval_agent_ids)
        self.world = self.action_wrapper.reset(self.world, self.scenario.eval_agent_ids)
        self.world = self.nn_observer.reset(self.world, self.scenario.eval_agent_ids)
        return self.nn_observer.observe(world=self.world, agents_to_observe=self.scenario.eval_agent_ids)

    def step(self, action):
        self.world = self.action_wrapper.action_to_behavior(world=self.world, action=action)
        self.world.step(self.step_time)
        return self.get_nstate_reward_action_tuple(world=self.world, controlled_agents=self.scenario.eval_agent_ids)

    @property
    def action_space(self):
        return self.action_wrapper.action_space

    @property
    def observation_space(self):
        return self.nn_observer.observation_space

    def get_nstate_reward_action_tuple(self, world, controlled_agents ):
        next_state = self.nn_observer.observe(world=self.world, agents_to_observe=controlled_agents)
        #reward, done, info = self.evaluator.get_evaluation(world=world)
        reward, done, info = (10, False, {"success": False})
        return next_state, reward, done, info


