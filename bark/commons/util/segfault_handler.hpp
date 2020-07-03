// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_SEGFAULT_HANDLER_HPP_
#define BARK_COMMONS_SEGFAULT_HANDLER_HPP_

#include <boost/stacktrace.hpp>
#include <iostream>

namespace bark {
namespace commons {

void SegfaultHandler(int sig) {
  std::cerr << boost::stacktrace::stacktrace() << std::endl;
  throw;
}

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_SEGFAULT_HANDLER_HPP_
