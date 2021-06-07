
try:
    import debug_settings
except:
    pass

import unittest
import pickle
import numpy as np

from bark.core.world.agent import *
from bark.core.models.behavior import *
from bark.core.world import *
from bark.core.world.map import *
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.core.world.goal_definition import *
from bark.core.world.evaluation import *
from bark.core.world.evaluation.ltl import *
from bark.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):

    def test_geometry_pickle(self):
        # point 2d
        p = Point2d(2 ,3)

        pa = pickle_unpickle(p)
        self.assertEqual(p.x(), pa.x())
        self.assertEqual(p.y(), pa.y())

        # linestring
        l = Line2d()
        l.AddPoint(p)
        l.AddPoint(Point2d(10,4))
        l.AddPoint(Point2d(1.555555, 1.244222))

        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.ToArray(), la.ToArray()))

        l = Line2d()
        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.ToArray(), la.ToArray()))

        # polygon
        p = CarLimousine()
        pa = pickle_unpickle(p)
        self.assertTrue(np.array_equal(p.ToArray(), pa.ToArray()))

        # polygon
        pr = CarRectangle()
        pra = pickle_unpickle(pr)
        self.assertTrue(np.array_equal(pr.ToArray(), pra.ToArray()))

    def test_behavior_model_pickle(self):
        
        params = ParameterServer()
        params["BehaviorIDMClassic"]["MaxAcceleration"] = 300.0
        b = BehaviorIDMClassic(params)

        ba = pickle_unpickle(b)
        self.assertTrue(isinstance(ba, BehaviorIDMClassic))
        self.assertEqual(ba.params.getReal("BehaviorIDMClassic::MaxAcceleration", "", 100), 300)

    def test_execution_model_pickle(self):
        
        params = ParameterServer()
        e = ExecutionModelInterpolate(params)

        ea = pickle_unpickle(e)
        self.assertTrue(isinstance(ea,ExecutionModelInterpolate))

    def test_dynamic_model_pickle(self):
        params = ParameterServer()
        d = SingleTrackModel(params)

        da = pickle_unpickle(d)
        self.assertTrue(isinstance(da, SingleTrackModel))


    def test_goal_definition(self):
        goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
        goal_definition = GoalDefinitionStateLimits(goal_polygon, (0.2 , 0.5))

        goal_definition_after = pickle_unpickle(goal_definition)

        self.assertTrue(np.array_equal(goal_definition.xy_limits.center, \
                                        goal_definition_after.xy_limits.center))


        goal_definition2 = GoalDefinitionStateLimits(goal_polygon, (0.2 , 0.5))
        goal_definition_sequential = GoalDefinitionSequential([goal_definition, goal_definition2])
        goal_definition_sequential_after = pickle_unpickle(goal_definition_sequential)

        sequential_goals_after = goal_definition_sequential_after.sequential_goals
        self.assertTrue(np.array_equal(sequential_goals_after[0].xy_limits.ToArray(), \
                                        goal_definition.xy_limits.ToArray()))

        self.assertTrue(np.array_equal(sequential_goals_after[1].xy_limits.ToArray(), \
                                        goal_definition2.xy_limits.ToArray()))
    
    def test_const_label_function(self):
        clf = ConstantLabelFunction("test")
        clf_after = pickle_unpickle(clf)
        self.assertEqual(clf.value, clf_after.value)

        dlf = SafeDistanceLabelFunction("test", False, 1.0, 1.0, -5.0, -5.0, True, 4, False, 1.0, False)
        dlf_after = pickle_unpickle(clf)

        tlf = DenseTrafficLabelFunction("test", 20.0, 8)
        tlf_after = pickle_unpickle(tlf)

        lclf = LaneChangeLabelFunction("test")
        lclf_after = pickle_unpickle(lclf)

    def test_agent_pickle(self):
        params = ParameterServer()
        behavior = BehaviorIDMClassic(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel(params)
        shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 5])
        goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
        goal_definition = GoalDefinitionPolygon(goal_polygon)
        agent = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), goal_definition )

        agent_after = pickle_unpickle(agent)

        self.assertEqual(agent_after.id , agent.id)
        self.assertTrue(np.array_equal(agent_after.state, agent.state) )
        self.assertTrue(np.array_equal(agent_after.goal_definition.goal_shape.center, \
                                       agent.goal_definition.goal_shape.center))

        goal_definition_2 = GoalDefinitionStateLimits(goal_polygon, (0.2 , 0.5))
        agent2 = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), goal_definition_2)

        agent_after2 = pickle_unpickle(agent2)

        self.assertEqual(agent_after2.id , agent2.id)
        self.assertTrue(np.array_equal(agent_after2.state, agent.state) )
        self.assertTrue(np.array_equal(agent_after2.goal_definition.xy_limits.center, \
                                       agent2.goal_definition.xy_limits.center))

        agent_list = []
        agent_list.append(agent)

        agent_list_after = pickle_unpickle(agent_list)

        self.assertEqual(agent_list_after[0].id , agent.id)
        self.assertTrue(np.array_equal(agent_list_after[0].state, agent.state) )

    def test_agent_pickle_uct_planner(self):
        try:
            from bark.core.models.behavior import BehaviorUCTSingleAgentMacroActions
        except:
            print("Rerun test with --define planner_uct=true")
            return

        params = ParameterServer()
        behavior = BehaviorUCTSingleAgentMacroActions(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel(params)
        shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 5])
        goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
        goal_definition = GoalDefinitionPolygon(goal_polygon)
        agent = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), goal_definition )

        agent_after = pickle_unpickle(agent)

        self.assertTrue(isinstance(agent_after.behavior_model, 
                                BehaviorUCTSingleAgentMacroActions))


    def test_behavior_safety(self):
      from bark.core.models.behavior import BehaviorSafety
      params = ParameterServer()
      pickle_unpickle(BehaviorSafety(params))

    def test_behavior_rss(self):
      from bark.core.models.behavior import BehaviorRSSConformant
      params = ParameterServer()
      rss_behavior = BehaviorRSSConformant(params)
      unpickled_rss_behavior = pickle_unpickle(rss_behavior)
      
      try:
        rss_behavior.SetLongitudinalResponse(1)
        rss_behavior.SetLateralLeftResponse(1)
        rss_behavior.SetLateralRightResponse(1)
        unpickled_rss_behavior = pickle_unpickle(rss_behavior)
        self.assertTrue(unpickled_rss_behavior.GetLongitudinalResponse() == 1)
        self.assertTrue(unpickled_rss_behavior.GetLateralLeftResponse() == 1)
        self.assertTrue(unpickled_rss_behavior.GetLateralRightResponse() == 1)
      except:
        print("Rerun test with --define rss=true")
      

      
if __name__ == '__main__':
    unittest.main()