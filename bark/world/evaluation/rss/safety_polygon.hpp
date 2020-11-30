// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_
#define BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_

#include <limits>
#include <memory>
#include <string>

#include "bark/geometry/polygon.hpp"
#include "bark/world/world.hpp"


namespace bark {
namespace world {
namespace evaluation {

using geometry::Polygon;
using bark::world::AgentId;

// TODO: make serializable
struct SafetyPolygon {
  double lat_left_safety_distance;
  double lat_right_safety_distance;
  double lon_safety_distance;
  Polygon polygon;
};

typedef std::shared_ptr<SafetyPolygon> SafetyPolygonPtr;

inline std::ostream &operator<<(std::ostream &os, const SafetyPolygon& v)
{
  os << "SafetyPolygon(";
  os << "lat_left_safety_distance:";
  os << v.lat_left_safety_distance;
  os << ",";
  os << "lat_right_safety_distance:";
  os << v.lat_right_safety_distance;
  os << ",";
  os << "lon_safety_distance:";
  os << v.lon_safety_distance;
  os << ")";
  return os;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_
