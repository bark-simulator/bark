// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle,
// Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
#define MODULES_COMMONS_SEGFAULT_HANDLER_HPP_

#include <stdio.h>
#include <execinfo.h>
#include <csignal>
#include <stdlib.h>
#include <unistd.h>


namespace modules {
namespace commons {

void SegfaultHandler(int sig) {
  void *array[35];
  size_t size;
  size = backtrace(array, 35);
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}


}  // namespace commons
}  // namespace modules


#endif  // MODULES_COMMONS_SEGFAULT_HANDLER_HPP_
