// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/opendrive/lane.hpp"

namespace bark {
namespace world {
namespace opendrive {

XodrLaneId XodrLane::lane_count = 0;

XodrLane::XodrLane()
    : lane_id_(++lane_count),
      lane_position_(0),
      link_(),
      line_(),
      lane_type_(XodrLaneType::NONE),
      driving_direction_(XodrDrivingDirection::FORWARD),
      road_mark_(),
      speed_() {}

XodrLane::XodrLane(const XodrLanePosition& lane_position)
    : lane_id_(++lane_count),
      lane_position_(lane_position),
      link_(),
      line_(),
      lane_type_(XodrLaneType::NONE),
      driving_direction_(XodrDrivingDirection::FORWARD),
      road_mark_(),
      speed_() {}

bool XodrLane::append(geometry::Line previous_line,
                      XodrLaneWidth lane_width_current, float s_inc) {
  geometry::Line tmp_line = CreateLineWithOffsetFromLine(
      previous_line, lane_position_, lane_width_current, s_inc);
  line_.AppendLinestring(tmp_line);
  return true;
}

}  // namespace opendrive
}  // namespace world
}  // namespace bark
