// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

XodrLaneId XodrLane::lane_count = 0;

float XodrLane::CurvatureAt(const float s, const float s_delta) const {
  /*
 geometry::Point2d p0 = GetPointAtS(line_, s - s_delta);
 geometry::Point2d p1 = GetPointAtS(line_, s + s_delta);
  float ds = distance(p0, p1);
  */
  return 0.0;
}
float XodrLane::CurvatureDotAt(const float s) const { return 0.0; }

float XodrLane::LaneWidthAt(const float s) const { return 0.0; }

float XodrLane::SFromPoint(const geometry::Point2d &point) const {
  return 0.0;  // modules::geometry::get_nearest_s(this->center_line_, point);
}

bool XodrLane::append(geometry::Line previous_line, XodrLaneWidth lane_width_current,
                  float s_inc) {
  geometry::Line tmp_line = CreateLineWithOffsetFromLine(
      previous_line, lane_position_, lane_width_current, s_inc);
  line_.append_linestring(tmp_line);
  return true;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
