// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_UTIL_HPP_
#define MODULES_COMMONS_UTIL_HPP_

#include <stdexcept>    // std::logic_error
#include <iostream>     // std::cerr
#include <boost/stacktrace.hpp>
#include <glog/logging.h>
#include "modules/commons/util/operators.hpp"


namespace modules {
namespace commons {

// ASSERTION
  inline void AssertionFailedMsg(char const* expr, char const* function, char const* file, long line) {
      std::cerr << "Expression '" << expr << "' is false in function '" << function << "'(" << file << ", l." << line <<  "): " << "\n"
          << "Backtrace:\n" << boost::stacktrace::stacktrace() << '\n';
  }

}  // namespace commons
}  // namespace modules

#define BARK_EXPECT_TRUE(cond) \
    do \
    { \
        if (!(cond)) \
        { \
            modules::commons::AssertionFailedMsg(#cond,__FUNCTION__, __FILE__, __LINE__); \
        } \
    } while(0)



#endif  // MODULES_COMMONS_UTIL_HPP_