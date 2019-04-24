import unittest
import pickle
import numpy as np

from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.geometry import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    f = open('temp.pickle','wb')
    pickle.dump(object, f)
    f.close()
    f = open( 'temp.pickle', "rb" )
    object = pickle.load( f)
    f.close()
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
        l.addPoint(p)
        l.addPoint(Point2d(10,4))
        l.addPoint(Point2d(1.555555, 1.244222))

        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.toArray(), la.toArray()))

        l = Line2d()
        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.toArray(), la.toArray()))

        # polygon
        p = CarLimousine()
        pa = pickle_unpickle(p)
        self.assertTrue(np.array_equal(p.toArray(), pa.toArray()))

    def test_behavior_model_pickle(self):
        
        params = ParameterServer()
        b = BehaviorConstantVelocity(params)

        ba = pickle_unpickle(b)
        self.assertTrue(isinstance(ba, BehaviorConstantVelocity))

    def test_execution_model_pickle(self):
        
        params = ParameterServer()
        e = ExecutionModelInterpolate(params)

        ea = pickle_unpickle(e)
        self.assertTrue(isinstance(ea,ExecutionModelInterpolate))

    def test_dynamic_model_pickle(self):
        
        params = ParameterServer()
        d = SingleTrackModel()

        da = pickle_unpickle(d)
        self.assertTrue(isinstance(da,SingleTrackModel))


    def test_agent_pickle(self):

        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel()
        shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 5])
        agent = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), lane_id = 0)

        agent_after = pickle_unpickle(agent)

        self.assertEqual(agent_after.id , agent.id)
        self.assertTrue(np.array_equal(agent_after.state, agent.state) )




if __name__ == '__main__':
    unittest.main()