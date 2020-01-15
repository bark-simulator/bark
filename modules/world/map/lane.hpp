// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_LANE_HPP_
#define MODULES_WORLD_MAP_LANE_HPP_

#include <memory>
#include <map>

#include "modules/world/opendrive/road.hpp"

namespace world {
namespace map {

using modules::opendrive::RoadPtr;
using modules::opendrive::Road;
using modules::opendrive::Lanes;
using modules::opendrive::Lane;

struct Boundary {
  Line line_;
  Type type_;
  // additional parameters: color, ...
};

// ONLY STORE STUFF
struct BarkLane : public Lane {
  explicit BarkLane(const LanePtr& lane) : Lane(lane) {}

  BarkLanePtr GetLeftLane(unisgned int lane_id) const {
    return left_lanes_.at(lane_id);
  }

  BarkLanePtr GetRightLane(unisgned int lane_id) const {
    return right_lanes_.at(lane_id);
  }

  BarkLanePtr GetNextLane() const {
    return next_lane_id_;
  }

  std::map<int, BarkLanePtr> left_lanes_;  // from_id, to_id
  std::map<int, BarkLanePtr> right_lanes_;  // from_id, to_id
  BarkLanePtr next_lane_;

  Line center_line_;
  Boundary left_boundary_;
  Boundary right_boundary_;
  Polygon polygon_;
};

using BarkLanePtr = std::shared_ptr<BarkLane>;
using BarkLanes = std::map<unsigned int, BarkLanePtr>;

}  // namespace map
}  // namespace world

#endif  // MODULES_WORLD_MAP_LANE_HPP_
