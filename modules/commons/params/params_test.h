// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_PARAMS_PARAMS_TEST_H_
#define MODULES_COMMONS_PARAMS_PARAMS_TEST_H_

#include "modules/commons/params/params.hpp"
#include <iostream>
#include <fstream>

namespace modules {
namespace commons {

inline void DoSomeParams(Params *params) {
  // CHILD TEST: DOES NOT WORK SINCE P IS NOT OF CLASS PARAMSERVER
  Params *params2 = params->AddChild("newchild");

  // test int, bool and real access with hierarchies
  std::cout << params2->get_int("hierarchy1::hierarchy2", "test", 40);
  std::cout << params2->get_bool("hierarchy1::hierarchy2-1::hierarchy3",
                                 "test2",
                                 false);
  std::cout << params2->get_real("param_cpp", "test param_cpp", 16.5);
}

}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_PARAMS_TEST_H_
