// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle,
// Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
#define MODULES_COMMONS_SEGFAULT_HANDLER_HPP_

#include <boost/stacktrace.hpp>

namespace modules {
namespace commons {

void SegfaultHandler(int sig) {
  std::cerr << boost::stacktrace::stacktrace() << std::endl;
}

}  // namespace commons
}  // namespace modules


#endif  // MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
