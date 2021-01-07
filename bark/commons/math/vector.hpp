// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_MATH_VECTOR_HPP_
#define BARK_COMMONS_MATH_VECTOR_HPP_

#include <numeric>
#include <vector>

namespace bark {
namespace commons {
namespace math {
inline double CalculateMean(const std::vector<double>& v) {
  size_t n = v.size(); 
  double mean = 0.0;
  if ( n != 0) {
      mean = std::accumulate( v.begin(), v.end(), 0.0) / n; 
  }
  return mean;
}

}  // namespace math
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_MATH_VECTOR_HPP_