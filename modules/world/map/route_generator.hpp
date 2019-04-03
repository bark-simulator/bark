// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_ROUTE_GENERATOR_HPP_
#define MODULES_WORLD_MAP_ROUTE_GENERATOR_HPP_

#include <vector>
#include <string>
#include <ostream>
#include <limits>
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/geometry/geometry.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LaneId;
using modules::world::opendrive::LanePtr;
using numeric_double_limits = std::numeric_limits<double>;
using namespace modules::geometry;

// TODO(@hart): refactor/concept
class RouteGenerator {
 public:
  explicit RouteGenerator(LaneId goal_lane_id,
                          const MapInterfacePtr& map_interface) :
    computed_(false),
    inner_line_(Line()),
    outer_line_(Line()),
    center_line_(Line()),
    map_interface_(map_interface),
    goal_lane_id_(goal_lane_id) {}

  RouteGenerator(const RouteGenerator& rg) :
    computed_(rg.computed_),
    inner_line_(rg.inner_line_),
    outer_line_(rg.outer_line_),
    center_line_(rg.center_line_),
    map_interface_(rg.map_interface_),
    goal_lane_id_(rg.goal_lane_id_) {}

  void set_goal_lane_id(LaneId goal_lane_id) { goal_lane_id_ = goal_lane_id_; }

  void set_map_interface(MapInterfacePtr map) { map_interface_ = map; }

  Line concatenate_lines(Line l0, Line l1);

  Line calculate_center_line(double horizon = numeric_double_limits::max());

  bool generate(Point2d point,
                LaneId goal_lane_id,
                double horizon = numeric_double_limits::max());

  Line get_inner_line() const { return inner_line_; }
  Line get_outer_line() const { return outer_line_; }
  Line get_center_line() const { return center_line_; }

 private:
  bool computed_;
  Line inner_line_, outer_line_, center_line_;
  MapInterfacePtr map_interface_;
  LaneId goal_lane_id_;
};

using RouteGeneratorPtr = std::shared_ptr<RouteGenerator>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_ROUTE_GENERATOR_HPP_
