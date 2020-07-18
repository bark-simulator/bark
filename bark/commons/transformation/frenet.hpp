// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_TRANSFORMATION_FRENET_POSITION_HPP_
#define BARK_COMMONS_TRANSFORMATION_FRENET_POSITION_HPP_

#include "bark/geometry/line.hpp"

namespace bark {
namespace commons {
namespace transformation {

struct FrenetPosition {
  FrenetPosition() : lon(0.0f), lat(0.0f) {}
  FrenetPosition(const double& longitudinal, const double& lateral)
      : lon(longitudinal), lat(lateral) {}
  FrenetPosition(const bark::geometry::Point2d& position,
                 const bark::geometry::Line& path);
  FrenetPosition operator+(const FrenetPosition& rhs);
  double lon;
  double lat;
};

inline std::ostream& operator<<(std::ostream& os, const FrenetPosition& frenet_position) {
    os << "FrenetPosition = (" << frenet_position.lon
       << " , " << frenet_position.lat
       << ")";
    return os;
}

}  // namespace transformation
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_TRANSFORMATION_FRENET_POSITION_HPP_