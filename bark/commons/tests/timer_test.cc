// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/timer/timer.hpp"
#include "gtest/gtest.h"

#include <chrono>
#include <thread>

using namespace bark::commons::timer;
using namespace std::chrono;

TEST(timer_test, timer_test50ms) {
  Timer timer = Timer();
  timer.Start();
  std::this_thread::sleep_for(50ms);
  double duration = timer.DurationInSeconds();
  EXPECT_NEAR(duration, 50e-3, 5e-3);
}

TEST(timer_test, timer_test10ms) {
  Timer timer = Timer();
  timer.Start();
  std::this_thread::sleep_for(10ms);
  double duration = timer.DurationInSeconds();
  EXPECT_NEAR(duration, 10e-3, 1e-3);
}
