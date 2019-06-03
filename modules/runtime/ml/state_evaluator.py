from bark.world.evaluation import EvaluatorGoalReached


class StateEvaluator:
    def get_evaluation(self, world):
        return # reward, done, info

    def add_world_evaluators(self, world)
        return # world


class GoalReached(StateEvaluator):
    def get_evaluation(self, world):

    def add_world_evaluators(self, world):
        evaluator = EvaluatorGoalReached(agent.id)
        world.add_evaluator("success", evaluator)
        

