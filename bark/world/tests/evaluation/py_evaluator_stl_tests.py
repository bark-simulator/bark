import unittest
from bark.world.evaluation.ltl import RobustnessSTL
from bark.world.objects import AgentId
from bark.runtime.scenario.scenario_generation.deterministic import DeterministicScenarioGeneration

class RobustnessSTLTests(unittest.TestCase):
    def setUp(self):
        param_server = ParameterServer(filename="bark/runtime/tests/data/deterministic_scenario.json")

        scenario_generation = DeterministicScenarioGeneration(num_scenarios=1,
                                                              random_seed=0,
                                                              params=param_server)

        scenario, idx = scenario_generation.get_next_scenario()
        self.observed_world = scenario.GetWorldState()
        self.agent_id = 1
        self.ltl_formula_str = "F label"
        self.label_functions = [ConstantLabelFunction("label")]

        self.evaluator = RobustnessSTL(self.agent_id, self.ltl_formula_str, self.label_functions)

    def test_evaluate(self):
        res = self.evaluator.Evaluate(self.observed_world)
        self.assertEqual(res, 0)

    def test_robustness(self):
        self.evaluator.Evaluate(self.observed_world)
        robustness = self.evaluator.CalculateRobustness()
        self.assertEqual(robustness, 1.0)

    def test_normalize_robustness(self):
        self.evaluator.Evaluate(self.observed_world)
        robustness = self.evaluator.CalculateRobustness()
        norm_robustness = self.evaluator.NormalizeRobustness(robustness)
        self.assertEqual(norm_robustness, 1.0)

if __name__ == '__main__':
    unittest.main()
