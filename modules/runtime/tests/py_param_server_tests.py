# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import pickle

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
    value_float = params_child["Child1"]["Child2"]["ValueFloat", "Desc", 2.0]
    value_bool_false = params_child["Child1"]["Child2"]["ValueBoolFalse", "Desc", False]
    value_bool_true = params_child["Child3"]["Child2"]["ValueBoolTrue", "Desc", True]
    value_int = params_child["Child1"]["Child4"]["ValueInt", "Desc", 2041]
    value_list_list_float = params_child["Child1"]["Child4"]["ValueListListFloat", "Desc", [[1.0,2.0, 5.4545234], [1.1266135,2.0], [123.234234]]]

    behavior = BehaviorIDMClassic(params_child)
    cpp_object = CppParamServerTestObject(params_child)
    cpp_unpickled = pickle_unpickle(cpp_object)
    params = cpp_unpickled.GetParams()


if __name__ == '__main__':
  unittest.main()
