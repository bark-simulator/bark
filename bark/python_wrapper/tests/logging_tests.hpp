// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_WRAPPING_TESTS_LOGGING_TESTS_HPP_
#define PYTHON_WRAPPING_TESTS_LOGGING_TESTS_HPP_

#include "glog/logging.h"

namespace py = pybind11;

namespace bark {
namespace commons {

void do_logging() {
  VLOG(1) << "Logged with verbosity 1";
  VLOG(2) << "Logged with verbosity 2";
  VLOG(3) << "Logged with verbosity 3";
  VLOG(4) << "Logged with verbosity 4";

  LOG(INFO) << "INFO log";

  LOG(WARNING) << "WARNING log";

  LOG(ERROR) << "ERROR log";

  VLOG_EVERY_N(2, 2) << "Logged with verbosity 2 every 2 calls";
}

}  // namespace commons
}  // namespace bark

#endif  // PYTHON_WRAPPING_TESTS_LOGGING_TESTS_HPP_