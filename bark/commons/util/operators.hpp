// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_UTIL_OPERATORS_HPP_
#define BARK_COMMONS_UTIL_OPERATORS_HPP_

#include <boost/variant.hpp>
#include <ostream>
#include <vector>

template <class T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
  os << "[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end();
       ++ii) {
    os << " ";
    os.operator<<(*ii);
  }
  os << " ]";
  return os;
}

template <>
inline std::ostream& operator<<<std::vector<float>>(
    std::ostream& os, const std::vector<std::vector<float>>& v) {
  os << "[";
  for (const auto sb : v) {
    os << "[";
    for (const auto e : sb) {
      os << " " << e;
    }
    os << " ]";
  }
  os << " ]";
  return os;
}

#endif  // BARK_COMMONS_UTIL_OPERATORS_HPP_