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

const double B_PI = 3.14159265358979323846;  ///< Pi

const double B_2PI = B_PI * 2.0;  // 2 * Pi

const double B_PI_2 = B_PI / 2.0;  // Pi / 2

const double B_PI_4 = B_PI / 4.0;  // Pi / 4

const double B_RAD2DEG = 180.0 / B_PI;  // convert from rad to degree

const double B_DEG2RAD = B_PI / 180.0;  // convert from degree to rad

inline double Norm0To2PI(const double& angle) {
  double normalized = std::fmod(angle, B_2PI);
  if (normalized < 0) {
    return normalized + B_2PI;
  }
  return normalized;
}

inline double NormToPI(double x){
    double normalized = fmod(x + B_PI, B_2PI);
    if (normalized < 0) {
      normalized += B_2PI;
    }
    return normalized - B_PI;
}

/**
 * @brief calculated absolute angle difference
 * 
 * @param angle1 in [-pi, pi]
 * @param angle2 in [-pi, pi]
 * @return double 
 */
inline double AngleDiff(const double& angle1, const double& angle2) {
  return abs(NormToPI(angle1) - NormToPI(angle2));
}

/**
 * @brief calculated signed angle difference
 * 
 * @param angle1 in [-pi, pi]
 * @param angle2 in [-pi, pi]
 * @return double 
 */
inline double SignedAngleDiff(const double& angle1, const double& angle2) {
  auto adiff = NormToPI(angle1 - angle2);
  return std::fmod(adiff + B_PI, B_2PI) - B_PI;
}

}  // namespace geometry
}  // namespace bark

#endif