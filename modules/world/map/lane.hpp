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

using modules::opendrive::XodrRoadPtr;
using modules::opendrive::XodrRoad;
using modules::opendrive::XodrLanes;
using modules::opendrive::XodrLane;

struct Boundary {
  Line line_;
  Type type_;
  // additional parameters: color, ...
};

// ONLY STORE STUFF
struct Lane : public XodrLane {
  explicit Lane(const XodrLanePtr& lane) : XodrLane(lane) {}

  LanePtr GetLeftLane(unisgned int lane_id) const {
    return left_lanes_.at(lane_id);
  }

  LanePtr GetRightLane(unisgned int lane_id) const {
    return right_lanes_.at(lane_id);
  }

  LanePtr GetNextLane() const {
    return next_lane_id_;
  }

  std::map<int, LanePtr> left_lanes_;  // from_id, to_id
  std::map<int, LanePtr> right_lanes_;  // from_id, to_id
  LanePtr next_lane_;

  Line center_line_;
  Boundary left_boundary_;
  Boundary right_boundary_;
  Polygon polygon_;
};

using LanePtr = std::shared_ptr<Lane>;
using Lanes = std::map<unsigned int, LanePtr>;

}  // namespace map
}  // namespace world

#endif  // MODULES_WORLD_MAP_LANE_HPP_
