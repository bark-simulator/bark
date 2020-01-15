// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_
#define MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

class XodrLaneSection {
 public:
  explicit XodrLaneSection(float s) : s_(s) {}
  ~XodrLaneSection() {}

  XodrLanes get_lanes() const { return lanes_; }

  XodrLanePtr get_lane_by_position(XodrLanePosition pos);

  XodrLanePtr get_nearest_lane_on_n(double x, double y, double vx, double vy);
  XodrLanePtr get_lane_with_offset(const models::dynamic::State& state,
                               double angle_offset);

  XodrLanePtr get_left_lane(const models::dynamic::State& state) {
    return get_lane_with_offset(state, 3.14 / 2);
  }
  XodrLanePtr get_right_lane(const models::dynamic::State& state) {
    return get_lane_with_offset(state, -3.14 / 2);
  }

  //! setter functions
  void add_lane(const XodrLanePtr& l);

  //! getter functions
  float get_s() const { return s_; }

 private:
  float s_;
  XodrLanes lanes_;
};

inline std::string print(const XodrLaneSection& ls) {
  std::stringstream ss;
  ss << "s: " << ls.get_s() << std::endl;
  for (auto const& l : ls.get_lanes())
    ss << "XodrLane: " << print(*(l.second)) << std::endl;
  return ss.str();
}

using XodrLaneSectionPtr = std::shared_ptr<XodrLaneSection>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_
