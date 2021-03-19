# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

try:
    import debug_settings
except:
    pass

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
    value_double = params_child["Child1"]["Child2"]["ValueFloat", "Desc", 2.0]
    value_bool_false = params_child["Child1"]["Child2"]["ValueBoolFalse", "Desc", False]
    value_bool_true = params_child["Child3"]["Child2"]["ValueBoolTrue", "Desc", True]
    value_int = params_child["Child1"]["Child4"]["ValueInt", "Desc", 2041]
    value_list_list_double = params_child["Child1"]["Child4"]["ValueListListFloat", "Desc",
         [[1.0, 2.0, float(5.4545234)], [float(1.1266135),2.0], [float(123.234234)]]]
    value_list_double = params_child["Child1"]["Child5"]["ValueListFloat", "Desc",
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

  def test_set_item_using_delimiter(self):
    params = ParameterServer()
    _ = params["test_child"]["Child2"]["ValueFloat", "Desc", 2.0]
    params["test_child::Child2::ValueFloat"] = 3.2323
    self.assertEqual(params["test_child"]["Child2"]["ValueFloat"], 3.2323)

    child = params.AddChild("test_child5::Child5")
    child["test_param2"] = "etesd99533sbgfgf"
    self.assertEqual(params["test_child5"]["Child5"]["test_param2", "Desc", 0], "etesd99533sbgfgf")

  def test_set_item_param_server_list_included_in_hierarchy(self):
    params = ParameterServer()
    params["test_child"]["Child2"]["ListOfParamServers"] = [{
      "ListEl1Param1" : 1.0,
      "ListEl1Param2" : 5,
      "ListEl1Child1" : {
        "ListEl1Child1Param1" : "dfdfdfasdgdfhdfg",
        "ListEl1Child1Param2" : 4343,
        
      },
      "EqualNameParams": {
        "EqualNameParam1": 20.03434,
        "EqualNameParam2": 2167767545
      }
    },
    {
      "ListEl2Param1" : 2.0,
      "ListEl2Param2" : 232,
      "EqualNameParams": {
        "EqualNameParam1": 20.03434,
        "EqualNameParam2": 2123232
      }
    }]

    self.assertEqual(params["test_child"]["Child2"]["ListOfParamServers"][0]["ListEl1Param2"], 5)
    self.assertTrue(isinstance(params["test_child"]["Child2"]["ListOfParamServers"][0], ParameterServer))
    self.assertEqual(params["test_child"]["Child2"]["ListOfParamServers"][1]["ListEl2Param1"], 2.0)
    self.assertEqual(params["test_child"]["Child2"]["ListOfParamServers"][0]["ListEl1Child1"]["ListEl1Child1Param1"], "dfdfdfasdgdfhdfg")

    params["test_child::Child2::ListOfParamServers::EqualNameParams::EqualNameParam1"] = 4545.232566
    self.assertEqual(params["test_child"]["Child2"]["ListOfParamServers"][0]["EqualNameParams"]["EqualNameParam1"], 4545.232566)
    self.assertEqual(params["test_child"]["Child2"]["ListOfParamServers"][1]["EqualNameParams"]["EqualNameParam1"], 4545.232566)

  def test_key_not_found(self):
    params = ParameterServer(log_if_default=True)
    params_child = params["test_child"]
    del params
    params_child["Child1"]["Child2"]["ValueFloat"] = 2.0
    params_child["Child1"]["Child10"]["ValueBoolFalse"] = False

    cpp_object = CppParamServerTestObject(params_child)

  def test_pickle_param_server(self):
    params = ParameterServer()
    params_child = params["test_child"]
    _  = params_child["Test1"]["Test2"]["Lala", "", False]
    _  = params_child["Test3"]["Test2"]["Lala1", "", 23.3434]
    _  = params_child["Test1"]["Test2"]["asdsd", "", 14]

    params_unpickled = pickle_unpickle(params)
    self.assertAlmostEquals(params_unpickled["test_child"]["Test1"]["Test2"]["Lala"], False)
    self.assertAlmostEquals(params_unpickled["test_child"]["Test3"]["Test2"]["Lala1"], 23.3434)
    self.assertAlmostEquals(params_unpickled["test_child"]["Test1"]["Test2"]["asdsd"], 14)

  def test_append_param_server(self):
    params1 = ParameterServer()
    params1["test1"]["test2"]["param1"] = 4.023343
    params1["test1"]["test2"]["test3"]["param1"] = 4234.00032356
    params1["test1"]["test2"]["param2"] = 2356

    params2 = ParameterServer()
    params2["test1"]["test2"]["test3"]["param2"] = 345554
    params2["test1"]["test2"]["param3"] = 52.0451646
    params2["test1"]["test2"]["param2"] = 123578

    params1_clone = params1.clone()

    params1_clone.AppendParamServer(params2, overwrite=True)
    self.assertEqual(params1_clone["test1"]["test2"]["param1"], 4.023343)
    self.assertEqual(params1_clone["test1"]["test2"]["test3"]["param1"], 4234.00032356)
    self.assertEqual(params1_clone["test1"]["test2"]["param2"], 123578)
    self.assertEqual(params1_clone["test1"]["test2"]["test3"]["param2"], 345554)
    self.assertEqual(params1_clone["test1"]["test2"]["param3"], 52.0451646)

    params2.AppendParamServer(params1, overwrite=True)
    self.assertEqual(params2["test1"]["test2"]["param1"], 4.023343)
    self.assertEqual(params2["test1"]["test2"]["test3"]["param1"], 4234.00032356)
    self.assertEqual(params2["test1"]["test2"]["param2"], 2356)
    self.assertEqual(params2["test1"]["test2"]["test3"]["param2"], 345554)
    self.assertEqual(params2["test1"]["test2"]["param3"], 52.0451646)

  def test_has_unequal_params(self):
    params1 = ParameterServer()
    params1["test1"]["test2"]["param1"] = 4.023343
    params1["test1"]["test2"]["test3"]["param1"] = 4234.00032356
    params1["test1"]["test2"]["param2"] = 2356

    params2 = ParameterServer()
    params2["test1"]["test2"]["test3"]["param2"] = 345554
    params2["test1"]["test2"]["param3"] = 52.0451646
    params2["test1"]["test2"]["param2"] = 123578

    params1_clone = params1.clone()
    params1_clone.AppendParamServer(params2, overwrite=True)

    result, unequal_params = params1_clone.HasEqualParamsAs(params2)
    self.assertFalse(result)
    self.assertEqual(len(unequal_params), 0)

    params1.AppendParamServer(params2, overwrite=False)
    result, unequal_params = params1.HasEqualParamsAs(params2)
    self.assertTrue(result)
    self.assertEqual(unequal_params["test1"]["test2"]["param2"], 123578)

if __name__ == '__main__':
  unittest.main()
