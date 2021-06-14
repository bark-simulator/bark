// Copyright (c) 2021 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_TIMER_TIMER_HPP_
#define BARK_COMMONS_TIMER_TIMER_HPP_

#include "boost/date_time/posix_time/posix_time.hpp"

namespace bark {
namespace commons {
namespace timer {

class Timer {
 public:
  Timer() : t1_() {}
  virtual ~Timer(){};
  inline void Start() { t1_ = boost::posix_time::microsec_clock::local_time(); }

  inline double DurationInSeconds() {
    boost::posix_time::ptime t2 =
        boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = t2 - t1_;
    return static_cast<double>(diff.total_microseconds()) / 1e6;
  }

 private:
  boost::posix_time::ptime t1_;
};

}  // namespace timer
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_TIMER_TIMER_HPP_