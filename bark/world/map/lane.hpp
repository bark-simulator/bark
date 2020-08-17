// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_MAP_LANE_HPP_
#define BARK_WORLD_MAP_LANE_HPP_

#include <map>
#include <memory>

#include "bark/geometry/geometry.hpp"
#include "bark/world/opendrive/road.hpp"

namespace bark {
namespace world {
namespace map {

using LaneId = unsigned int;
using bark::geometry::Line;
using bark::geometry::Polygon;
using bark::world::opendrive::XodrLane;
using bark::world::opendrive::XodrLanePtr;
using bark::world::opendrive::XodrLanes;
using bark::world::opendrive::XodrRoad;
using bark::world::opendrive::XodrRoadMark;
using bark::world::opendrive::XodrRoadPtr;

struct Boundary {
  Line GetLine() const { return line_; }
  XodrRoadMark GetType() const { return type_; }
  void SetLine(const Line& line) { line_ = line; }
  void SetType(const XodrRoadMark& type) { type_ = type; }
  Line line_;
  XodrRoadMark type_;
};

struct Lane : public XodrLane {
  explicit Lane(const XodrLanePtr& lane)
      : XodrLane(lane), next_lane_(nullptr) {}

  //! Getter
  std::shared_ptr<Lane>& GetNextLane() { return next_lane_; }
  Boundary& GetLeftBoundary() { return left_boundary_; }
  Boundary& GetRightBoundary() { return right_boundary_; }
  std::shared_ptr<Lane> GetLeftLane() const { return left_lane_.lock(); }
  std::shared_ptr<Lane> GetRightLane() const { return right_lane_.lock(); }
  Line& GetCenterLine() { return center_line_; }
  Polygon& GetPolygon() { return polygon_; }

  //! Setter
  void SetNextLane(const std::shared_ptr<Lane>& lane) { next_lane_ = lane; }
  void SetPolygon(const Polygon& poly) { polygon_ = poly; }
  void SetCenterLine(const Line& line) { center_line_ = line; }
  void SetLeftBoundary(const Boundary& boundary) { left_boundary_ = boundary; }
  void SetRightBoundary(const Boundary& boundary) {
    right_boundary_ = boundary;
  }
  void SetLeftLane(const std::shared_ptr<Lane>& left_lane) {
    left_lane_ = left_lane;
  }
  void SetRightLane(const std::shared_ptr<Lane>& right_lane) {
    right_lane_ = right_lane;
  }

  bool operator==(const Lane& rhs) const {
    return center_line_ == rhs.center_line_;
  }
  bool operator!=(const Lane& rhs) const { return !(rhs == *this); }

  std::weak_ptr<Lane> left_lane_;   // from_id, to_id
  std::weak_ptr<Lane> right_lane_;  // from_id, to_id
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
}  // namespace bark

#endif  // BARK_WORLD_MAP_LANE_HPP_
