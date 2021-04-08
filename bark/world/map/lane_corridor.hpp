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
namespace bg = boost::geometry;

struct LaneCorridor {
  using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;
  //! Getter
  Line& GetLeftBoundary() { return left_boundary_; }
  Line& GetRightBoundary() { return right_boundary_; }
  Line& GetCenterLine() { return center_line_; }
  Line& GetFineCenterLine() { return fine_center_line_; }
  Polygon& GetMergedPolygon() { return merged_polygon_; }
  std::map<double, LanePtr>& GetLanes() { return lanes_; }
  LanePtr GetCurrentLane(const Point2d& pt) const {
    // NOTE(@all): the matching could also be done using the s-value
    for (auto& lane : lanes_) {
      if (Within(pt, lane.second->GetPolygon())) return lane.second;
    }
    return nullptr;
  }
  double GetLength() const { return center_line_.Length(); }
  double GetS(const Point2d& pt) const {
    return std::get<1>(GetNearestPointAndS(center_line_, pt));
  }
  double LengthUntilEnd(const Point2d& pt) const {
    return GetLength() - GetS(pt);
  }

  double GetLaneWidth(const Point2d& pt) {
    uint idx = FindNearestIdx(GetCenterLine(), pt);
    // assumption: center, left and right have same # elements
    Point2d left_pt = GetPointAtIdx(GetLeftBoundary(), idx);
    Point2d right_pt = GetPointAtIdx(GetRightBoundary(), idx);
    double width = bark::geometry::Distance(left_pt, right_pt);
    return width;
  }

  //! Setter
  void SetLeftBoundary(const Line& boundary) { left_boundary_ = boundary; }
  void SetRightBoundary(const Line& boundary) { right_boundary_ = boundary; }
  void SetCenterLine(const Line& line) { center_line_ = line; }
  void SetFineCenterLine(const Line& line) { fine_center_line_ = line; }
  void SetMergedPolygon(const Polygon& poly) { merged_polygon_ = poly; }
  void SetLane(double s_start, const LanePtr& lane) { lanes_[s_start] = lane; }

  bool operator==(const LaneCorridor other) {
    return lanes_.size() == other.lanes_.size() &&
           std::equal(lanes_.begin(), lanes_.end(), other.lanes_.begin(),
                      [](const std::map<double, LanePtr>::value_type& a,
                         const std::map<double, LanePtr>::value_type& b) {
                        return a.first == b.first && *(a.second) == *(b.second);
                      });
  }

  bool operator!=(const LaneCorridor other) { return !(*this == other); }

  std::map<double, LanePtr> lanes_;  // s_end, LanePtr
  Line center_line_;
  Line fine_center_line_;
  Polygon merged_polygon_;
  Line left_boundary_;
  Line right_boundary_;
};
using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;

inline std::ostream& operator<<(std::ostream& os, LaneCorridor& lc) {
  Line centerline = lc.GetCenterLine();
  Point2d center_front_pt = centerline.obj_.front();
  Point2d center_last_pt = centerline.obj_.back();
  os << "LaneCorridor = ("
    << " Length: " << lc.GetLength() << ", "
    << " center(0): [" << boost::geometry::get<0>(center_front_pt) << ", " << boost::geometry::get<1>(center_front_pt) << "], " 
    << " center(end): [" << boost::geometry::get<0>(center_last_pt) << ", " << boost::geometry::get<1>(center_last_pt) << "]"
    << " width(0): " << lc.GetLaneWidth(center_front_pt) << ", " 
    << " width(end): " << lc.GetLaneWidth(center_last_pt) << ")";
  return os;
}


}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_MAP_LANE_CORRIDOR_HPP_
