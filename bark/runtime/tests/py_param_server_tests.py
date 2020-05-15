# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import pickle
import numpy as np

from bark.runtime.commons.parameters import ParameterServer
from bark.core.commons import CppParamServerTestObject
from bark.core.models.behavior import BehaviorIDMClassic

def pickle_unpickle(object):
    return pickle.loads(
      pickle.dumps(object))



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
    value_list_float = params_child["Child1"]["Child5"]["ValueListFloat", "Desc",
         [1.0, 2.0, float(5.4545234), float(1.1266135),2.0, float(123.234234)]]

    behavior = BehaviorIDMClassic(params_child)
    cpp_object = CppParamServerTestObject(params_child)
    cpp_unpickled = pickle_unpickle(cpp_object)

    self.assertEqual(cpp_object.GetRealValue(), 2.0)
    self.assertEqual(cpp_object.GetBoolValueFalse(), False)
    self.assertEqual(cpp_object.GetBoolValueTrue(), True)
    self.assertEqual(cpp_object.GetIntValue(), 2041)
    list1 = cpp_object.GetListListFloatValue()
    list2 =[[1.0,2.0, float(5.4545234)], [float(1.1266135),2.0], [float(123.234234)]]

    list3 = cpp_object.GetListFloatValue()
    list4 =[1.0, 2.0, float(5.4545234), float(1.1266135),2.0, float(123.234234)]

    self.assertEqual(len(list1), len(list2))
    for idx, _ in enumerate(list1):
      self.assertEqual(len(list1[idx]), len(list2[idx]))
      for idx2, _ in enumerate(list1[idx]):
        self.assertAlmostEqual(list1[idx][idx2], list2[idx][idx2], places=5)

    self.assertEqual(len(list3), len(list4))
    for idx, _ in enumerate(list3):
        self.assertAlmostEqual(list3[idx], list4[idx], places=5)

  def test_key_not_found(self):
    params = ParameterServer(log_if_default=True)
    params_child = params["test_child"]
    del params
    params_child["Child1"]["Child2"]["ValueFloat"] = 2.0
    params_child["Child1"]["Child10"]["ValueBoolFalse"] = False

    cpp_object = CppParamServerTestObject(params_child)



if __name__ == '__main__':
  unittest.main()
