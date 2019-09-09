// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_DRIVING_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_DRIVING_CORRIDOR_HPP_

#include "modules/geometry/line.hpp"
#include "modules/geometry/geometry.hpp"
#include "modules/world/map/frenet.hpp"

namespace modules
{
namespace world
{
namespace map
{
using namespace modules::geometry;

struct DrivingCorridor
{
  DrivingCorridor() : outer(Line()),
                      inner(Line()),
                      center(Line()),
                      computed(false) {}

  DrivingCorridor(const Line &outer, const Line &inner, const Line &center) : outer(outer),
                                                                              inner(inner),
                                                                              center(center) {}
  //! getter
  Line get_outer() const { return outer; }
  Line get_inner() const { return inner; }
  Line get_center() const { return center; }

  std::vector<std::pair<int, LaneId>> get_lane_ids() const
  {
    return lane_ids_;
  }

  //! getter
  void set_outer(const Line &o) { outer = o; }
  void set_inner(const Line &o) { inner = o; }
  void set_center(const Line &o) { center = o; }

  // returns Frenet coordinate to center line
  Frenet FrenetFromCenterLine(const Point2d &point) const { return Frenet(point, center); }
  Polygon CorridorPolygon() const
  {
    Line line = get_outer();
    line.reverse(); 
    line.append_linestring(get_inner());
    return Polygon(Pose(0, 0, 0), line);
  }

  bool intersects_polygon(const Polygon &polygon) const;

  Line outer, inner, center;
  // 1st entry is the index from where the 2nd value lane id is valid
  std::vector<std::pair<int, LaneId>> lane_ids_;
  bool computed;
};

using DrivingCorridorPtr = std::shared_ptr<DrivingCorridor>;

} // namespace map
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_MAP_DRIVING_CORRIDOR_HPP_