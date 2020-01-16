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
using modules::world::opendrive::XodrRoadMark;
using modules::world::opendrive::XodrLane;
using modules::world::opendrive::XodrLanePtr;
using modules::geometry::Polygon;
using modules::geometry::Line;


struct Boundary {
  void SetLine(const Line& line) {
    line_ = line;
  }
  void SetType(const XodrRoadMark& type) {
    type_ = type;
  }
  Line line_;
  XodrRoadMark type_;
};


struct Lane : public XodrLane {
  explicit Lane(const XodrLanePtr& lane) : XodrLane(lane) {}

  //! Getter
  std::shared_ptr<Lane> GetNextLane() const {
    return next_lane_;
  }
  Boundary GetLeftBoundary() const {
    return left_boundary_;
  }
  Boundary GetRightBoundary() const {
    return right_boundary_;
  }
  std::shared_ptr<Lane> GetLeftLane(const LaneId& lane_id) const {
    return left_lanes_.at(lane_id);
  }
  std::shared_ptr<Lane> GetRightLane(const LaneId& lane_id) const {
    return right_lanes_.at(lane_id);
  }
  Line GetCenterLine() const {
    return center_line_;
  }
  Polygon GetPolygon() const {
    return polygon_;
  }


  //! Setter
  void SetNextLane(const std::shared_ptr<Lane>& lane ) {
    next_lane_ = lane;
  }
  void SetPolygon(const Polygon& poly) {
    polygon_ = poly;
  }
  void SetCenterLine(const Line& line) {
    center_line_ = line;
  }
  void SetLeftBoundary(const Boundary& boundary) {
    left_boundary_ = boundary;
  }
  void SetRightBoundary(const Boundary& boundary) {
    right_boundary_ = boundary;
  }
  void SetLeftLane(
    const LaneId& lane_id,
    const std::shared_ptr<Lane>& left_lane) {
    left_lanes_[lane_id] = left_lane;
  }
  void SetRightLane(
    const LaneId& lane_id,
    const std::shared_ptr<Lane>& right_lane) {
    right_lanes_[lane_id] = right_lane;
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
