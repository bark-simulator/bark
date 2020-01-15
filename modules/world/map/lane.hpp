// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_LANE_HPP_
#define MODULES_WORLD_MAP_LANE_HPP_

#include <memory>
#include <map>

#include "modules/world/opendrive/road.hpp"
#include "modules/geometry/geometry.hpp"

namespace modules {
namespace world {
namespace map {

using LaneId = unsigned int;
using modules::world::opendrive::XodrRoadPtr;
using modules::world::opendrive::XodrRoad;
using modules::world::opendrive::XodrLanes;
using modules::world::opendrive::XodrLane;
using modules::world::opendrive::XodrLanePtr;
using modules::geometry::Polygon;
using modules::geometry::Line;


struct Boundary {
  Line line_;
  int type_;
};


struct Lane : public XodrLane {
  explicit Lane(const XodrLanePtr& lane) : XodrLane(lane) {}

  std::shared_ptr<Lane> GetLeftLane(LaneId lane_id) const {
    return left_lanes_.at(lane_id);
  }

  std::shared_ptr<Lane> GetRightLane(LaneId lane_id) const {
    return right_lanes_.at(lane_id);
  }

  std::shared_ptr<Lane> GetNextLane() const {
    return next_lane_;
  }

  void GetNextLane(const std::shared_ptr<Lane>& lane ) {
    next_lane_ = lane;
  }
  
  Boundary GetLeftBoundary() const {
    return left_boundary_;
  }

  Boundary GetRightBoundary() const {
    return right_boundary_;
  }



  std::map<int, std::shared_ptr<Lane>> left_lanes_;  // from_id, to_id
  std::map<int, std::shared_ptr<Lane>> right_lanes_;  // from_id, to_id
  std::shared_ptr<Lane> next_lane_;

  Line center_line_;
  Boundary left_boundary_;
  Boundary right_boundary_;
  Polygon polygon_;
};

using LanePtr = std::shared_ptr<Lane>;
using Lanes = std::map<LaneId, LanePtr>;

}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_LANE_HPP_
