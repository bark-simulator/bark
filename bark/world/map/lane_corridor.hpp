// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_MAP_LANE_CORRIDOR_HPP_
#define BARK_WORLD_MAP_LANE_CORRIDOR_HPP_

#include <boost/functional/hash.hpp>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "bark/geometry/geometry.hpp"
#include "bark/world/map/lane.hpp"
#include "bark/world/map/road.hpp"
#include "bark/world/opendrive/opendrive.hpp"

namespace bark {
namespace world {
namespace map {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Within;
using bark::world::opendrive::XodrRoadId;

struct LaneCorridor {
  using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;
  //! Getter
  Line& GetLeftBoundary() { return left_boundary_; }
  Line& GetRightBoundary() { return right_boundary_; }
  Line& GetCenterLine() { return center_line_; }
  Polygon& GetMergedPolygon() { return merged_polygon_; }
  std::map<float, LanePtr>& GetLanes() { return lanes_; }
  LanePtr GetCurrentLane(const Point2d& pt) const {
    // NOTE(@all): the matching could also be done using the s-value
    for (auto& lane : lanes_) {
      if (Within(pt, lane.second->GetPolygon())) return lane.second;
    }
    return nullptr;
  }
  float GetLength() const { return center_line_.Length(); }
  float GetS(const Point2d& pt) const {
    return std::get<1>(GetNearestPointAndS(center_line_, pt));
  }
  float LengthUntilEnd(const Point2d& pt) const {
    return GetLength() - GetS(pt);
  }

  float GetLaneWidth(const Point2d& pt) {
    uint idx = FindNearestIdx(GetCenterLine(), pt);
    // assumption: center, left and right have same # elements
    Point2d left_pt = GetPointAtIdx(GetLeftBoundary(), idx);
    Point2d right_pt = GetPointAtIdx(GetRightBoundary(), idx);
    float width = bark::geometry::Distance(left_pt, right_pt);
    return width;
  }

  //! Setter
  void SetLeftBoundary(const Line& boundary) { left_boundary_ = boundary; }
  void SetRightBoundary(const Line& boundary) { right_boundary_ = boundary; }
  void SetCenterLine(const Line& line) { center_line_ = line; }
  void SetMergedPolygon(const Polygon& poly) { merged_polygon_ = poly; }
  void SetLane(float s_start, const LanePtr& lane) { lanes_[s_start] = lane; }

  bool operator==(const LaneCorridor other) {
    return lanes_.size() == other.lanes_.size() &&
           std::equal(lanes_.begin(), lanes_.end(), other.lanes_.begin(),
                      [](const std::map<float, LanePtr>::value_type& a,
                         const std::map<float, LanePtr>::value_type& b) {
                        return a.first == b.first && *(a.second) == *(b.second);
                      });
  }

  bool operator!=(const LaneCorridor other) { return !(*this == other); }

  std::map<float, LanePtr> lanes_;  // s_end, LanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Line left_boundary_;
  Line right_boundary_;
};
using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;

}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_MAP_LANE_CORRIDOR_HPP_
