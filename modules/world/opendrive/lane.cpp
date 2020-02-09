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


bool XodrLane::append(geometry::Line previous_line, XodrLaneWidth lane_width_current,
                  float s_inc) {
  geometry::Line tmp_line = CreateLineWithOffsetFromLine(
      previous_line, lane_position_, lane_width_current, s_inc);
  line_.AppendLinestring(tmp_line);
  return true;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
