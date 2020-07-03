# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
from bark.core.commons import *
from bark.runtime.commons.parameters import ParameterServer


class ParamsTests(unittest.TestCase):
    def test_parameters(self):
        # initialize Params
        p = ParameterServer()

        # set new parameter
        self.assertTrue(p["LetsTest"]["hierarchy", "bla", True])

        # check whether right value is recovered
        tester = p["Car"]["Length", "Car Length", 6]
        tester2 = p["Car"]["Length", "Car Length", 8]
        self.assertEqual(tester, 6)
        self.assertEqual(
            tester2,
            6)  # value should not change, since key already exists in dict

        # check whether access without description and default value is possible
        self.assertEqual(p["Car"]["Length"], 6)

        # check whether setting values works
        p["Age"] = 24
        self.assertEqual(p["Age"], 24)
        p["Localization"]["Number of Particles"] = 2000
        self.assertEqual(p["Localization"]["Number of Particles"], 2000)

        # C++ Test in /bark/commons/Params/params_test.h
        # write in parameters in C++ and check whether they can be accessed in python afterwards
        #ParamsTest(p)
        #self.assertEqual(p["param_cpp"], 16.5)

        # add child in python
        child = p.AddChild("ch")
        self.assertTrue(child["ChildTest"]["hierarchy", "bla", True])

        # write parameters to json file
        p.Save("written_a_param_test.json")


if __name__ == '__main__':
    unittest.main()
