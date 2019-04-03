// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

LaneId Lane::lane_count = 0;

float Lane::curvature_at(const float s, const float s_delta) const {
  /*
 geometry::Point2d p0 = get_point_at_s(line_, s - s_delta);
 geometry::Point2d p1 = get_point_at_s(line_, s + s_delta);
  float ds = distance(p0, p1);
  */
  return 0.0;
}
float Lane::curvature_dot_at(const float s) const {
  return 0.0;
}

float Lane::lane_width_at(const float s) const {
  return 0.0;
}

float Lane::s_from_point(const geometry::Point2d &point) const {
  return 0.0; //modules::geometry::get_nearest_s(this->center_line_, point);
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
