// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_GEOMETRY_ANGLE_HPP_
#define BARK_GEOMETRY_ANGLE_HPP_

#include <cmath>

namespace bark {
namespace geometry {

const float B_PI = 3.14159265358979323846;  ///< Pi

const float B_2PI = B_PI * 2.0;  // 2 * Pi

const float B_PI_2 = B_PI / 2.0;  // Pi / 2

const float B_PI_4 = B_PI / 4.0;  // Pi / 4

const float B_RAD2DEG = 180.0 / B_PI;  // convert from rad to degree

const float B_DEG2RAD = B_PI / 180.0;  // convert from degree to rad

inline float Norm0To2PI(const float& angle) {
  float normalized = std::fmod(angle, B_2PI);
  if (normalized < 0) {
    return normalized + B_2PI;
  }
  return normalized;
}

inline float AngleDiff(const float& angle1, const float& angle2) {
  return abs(Norm0To2PI(angle1) - Norm0To2PI(angle2));
}

inline float SignedAngleDiff(const float& angle1, const float& angle2) {
  auto adiff = angle1 - angle2;
  return std::fmod(adiff + B_PI, B_2PI) - B_PI;
}

}  // namespace geometry
}  // namespace bark

#endif