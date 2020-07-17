import unittest

from bark.core.world import *
from bark.core.world.evaluation.ltl import *
from bark.runtime.commons.parameters import ParameterServer

from bark.runtime.scenario.scenario_generation.deterministic import DeterministicScenarioGeneration


class EvaluatorLTLTests(unittest.TestCase):
    def setUp(self):
        param_server = ParameterServer(
            filename="bark/runtime/tests/data/deterministic_scenario.json")

        scenario_generation = DeterministicScenarioGeneration(num_scenarios=1,
                                                              random_seed=0,
                                                              params=param_server)

        scenario, idx = scenario_generation.get_next_scenario()
        self.world = scenario.GetWorldState()
        self.agent_id = 1
        self.label = ConstantLabelFunction("label")

    def test_safety(self):
        evaluator = EvaluatorLTL(1, "G label", [self.label])
        self.world.AddEvaluator("rule", evaluator)
        res = self.world.Evaluate()
        self.assertEqual(res["rule"], 0)
        res = self.world.Evaluate()
        self.assertEqual(res["rule"], 0)
        self.label.value = False
        res = self.world.Evaluate()
        # Now violated
        self.assertEqual(res["rule"], 1)
        self.label.value = True
        res = self.world.Evaluate()
        # Violation should be persistent
        self.assertEqual(res["rule"], 1)
        self.label.value = False
        res = self.world.Evaluate()
        # Check for second violation
        self.assertEqual(res["rule"], 2)
        self.label.value = True
        res = self.world.Evaluate()
        # Violation should be persistent
        self.assertEqual(res["rule"], 2)

    def test_safety_violation(self):
        evaluator = EvaluatorLTL(1, "G !label", [self.label])
        self.world.AddEvaluator("rule", evaluator)
        res = self.world.Evaluate()
        self.assertEqual(res["rule"], 1)

    def test_guarantee(self):
        evaluator = EvaluatorLTL(1, "F label", [self.label])
        self.world.AddEvaluator("rule", evaluator)
        self.label.value = False
        res = self.world.Evaluate()
        # Expect a violation since the rule would be violated if the trace
        # would end here
        self.assertEqual(res["rule"], 1)
        self.label.value = True
        res = self.world.Evaluate()
        # Previous violation is not counted as the trace has not ended
        self.assertEqual(res["rule"], 0)
        res = self.world.Evaluate()
        self.assertEqual(res["rule"], 0)
        self.label.value = False
        res = self.world.Evaluate()
        # Should still be satisfied since the trace contained label = True
        self.assertEqual(res["rule"], 0)

    def test_guarantee_violation(self):
        evaluator = EvaluatorLTL(1, "F !label", [self.label])
        self.world.AddEvaluator("rule", evaluator)
        res = self.world.Evaluate()
        self.assertEqual(res["rule"], 1)

    def test_agent_relative_rule(self):
        evaluator = EvaluatorLTL(0, "G behind#0", [BehindOfLabelFunction("behind")])
        self.world.AddEvaluator("behind_rule", evaluator)
        res = self.world.Evaluate()
        self.assertEqual(res["behind_rule"], 0)
        # Now remove agent 1
        self.world.RemoveAgentById(1)
        res = self.world.Evaluate()
        # Rule should still be satisfied since there is no agent to be behind of
        self.assertEqual(res["behind_rule"], 0)


if __name__ == '__main__':
    unittest.main()
