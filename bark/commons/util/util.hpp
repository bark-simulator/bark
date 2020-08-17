// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_UTIL_HPP_
#define BARK_COMMONS_UTIL_HPP_

#include <glog/logging.h>
#include <boost/stacktrace.hpp>
#include <iostream>   // std::cerr
#include <stdexcept>  // std::logic_error
#include "bark/commons/util/operators.hpp"

namespace bark {
namespace commons {

// ASSERTION
inline void AssertionFailedMsg(char const* expr, char const* function,
                               char const* file, long line) {
  std::cerr << "Expression '" << expr << "' is false in function '" << function
            << "'(" << file << ", l." << line << "): "
            << "\n"
            << "Backtrace:\n"
            << boost::stacktrace::stacktrace() << '\n';
}

}  // namespace commons
}  // namespace bark

#define BARK_EXPECT_TRUE(cond)                                         \
  do {                                                                 \
    if (!(cond)) {                                                     \
      bark::commons::AssertionFailedMsg(#cond, __FUNCTION__, __FILE__, \
                                        __LINE__);                     \
    }                                                                  \
  } while (0)

#endif  // BARK_COMMONS_UTIL_HPP_