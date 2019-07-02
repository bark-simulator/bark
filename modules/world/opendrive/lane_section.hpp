// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_
#define MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_

#include <vector>
#include <map>

#include "modules/world/opendrive/lane.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace world {
namespace opendrive {


class LaneSection {
 public:
  explicit LaneSection(float s) : s_(s) {}
  ~LaneSection() {}

  Lanes get_lanes() const {return lanes_;}

  LanePtr get_lane_by_position(LanePosition pos);

  LanePtr get_nearest_lane_on_n(double x, double y, double vx, double vy);
  LanePtr get_lane_with_offset(const models::dynamic::State& state, double angle_offset);
  
  LanePtr get_left_lane(const models::dynamic::State& state) {
    return get_lane_with_offset(state, 3.14/2);
  }
  LanePtr get_right_lane(const models::dynamic::State& state) {
    return get_lane_with_offset(state, -3.14/2);
  }

  //! setter functions
  void add_lane(const LanePtr& l);

  //! getter functions
  float get_s() const { return s_; }

 private:
  float s_;
  Lanes lanes_;
};

using LaneSectionPtr = std::shared_ptr<LaneSection>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_LANE_SECTION_HPP_
