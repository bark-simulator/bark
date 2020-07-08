// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_PARAMS_PARAMS_TEST_H_
#define BARK_COMMONS_PARAMS_PARAMS_TEST_H_

#include <fstream>
#include <iostream>
#include "bark/commons/params/params.hpp"

namespace bark {
namespace commons {

inline void DoSomeParams(const ParamsPtr params) {
  // CHILD TEST: DOES NOT WORK SINCE P IS NOT OF CLASS PARAMSERVER
  auto params2 = params->AddChild("newchild");

  // test int, bool and real access with hierarchies
  std::cout << params2->GetInt("hierarchy1::hierarchy2", "test", 40);
  std::cout << params2->GetBool("hierarchy1::hierarchy2-1::hierarchy3", "test2",
                                false);
  std::cout << params2->GetReal("param_cpp", "test param_cpp", 16.5);
}

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_PARAMS_PARAMS_TEST_H_
