// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
#define MODULES_COMMONS_SEGFAULT_HANDLER_HPP_

#include <boost/stacktrace.hpp>
#include <iostream>

namespace modules {
namespace commons {

void SegfaultHandler(int sig) {
  std::cerr << boost::stacktrace::stacktrace() << std::endl;
}

}  // namespace commons
}  // namespace modules


#endif  // MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
