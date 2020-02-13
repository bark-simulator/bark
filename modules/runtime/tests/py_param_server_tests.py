# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import pickle
import numpy as np

from modules.runtime.commons.parameters import ParameterServer
from bark.commons import CppParamServerTestObject
from bark.models.behavior import BehaviorIDMClassic

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object



class ParamServerTests(unittest.TestCase):
  def test_pickle(self):
    params = ParameterServer()
    params_child = params["test_child"]
    del params
    value_float = params_child["Child1"]["Child2"]["ValueFloat", "Desc", 2.0]
    value_bool_false = params_child["Child1"]["Child2"]["ValueBoolFalse", "Desc", False]
    value_bool_true = params_child["Child3"]["Child2"]["ValueBoolTrue", "Desc", True]
    value_int = params_child["Child1"]["Child4"]["ValueInt", "Desc", 2041]
    value_list_list_float = params_child["Child1"]["Child4"]["ValueListListFloat", "Desc",
         [[1.0, 2.0, float(5.4545234)], [float(1.1266135),2.0], [float(123.234234)]]]

    behavior = BehaviorIDMClassic(params_child)
    cpp_object = CppParamServerTestObject(params_child)
    cpp_unpickled = pickle_unpickle(cpp_object)

    self.assertEqual(cpp_object.GetRealValue(), 2.0)
    self.assertEqual(cpp_object.GetBoolValueFalse(), False)
    self.assertEqual(cpp_object.GetBoolValueTrue(), True)
    self.assertEqual(cpp_object.GetIntValue(), 2041)
    list1 = cpp_object.GetListListFloatValue()
    list2 =[[1.0,2.0, float(5.4545234)], [float(1.1266135),2.0], [float(123.234234)]]

    self.assertEqual(len(list1), len(list2))
    for idx, _ in enumerate(list1):
      self.assertEqual(len(list1[idx]), len(list2[idx]))
      for idx2, _ in enumerate(list1[idx]):
        self.assertAlmostEqual(list1[idx][idx2], list2[idx][idx2], places=5)



if __name__ == '__main__':
  unittest.main()
