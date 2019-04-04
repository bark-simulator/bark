// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_local_map_HPP_
#define MODULES_WORLD_MAP_local_map_HPP_

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

struct DrivingCorridor {
  DrivingCorridor() : outer(Line()),
                      inner(Line()),
                      center(Line()){

  }
  Line outer, inner, center;
  std::vector<int> lane_ids_;

  // TODO(@fortiss): what IF functions would we like here
};

class LocalMap {
 public:
  explicit LocalMap(LaneId goal_lane_id,
                          const MapInterfacePtr& map_interface) :
    current_driving_corridor_(DrivingCorridor()),
    map_interface_(map_interface),
    goal_lane_id_(goal_lane_id) {}

  LocalMap(const LocalMap& lm) :
    current_driving_corridor_(lm.current_driving_corridor_),
    map_interface_(lm.map_interface_),
    goal_lane_id_(lm.goal_lane_id_) {}

  void set_goal_lane_id(LaneId goal_lane_id) { goal_lane_id_ = goal_lane_id_; }

  void set_map_interface(MapInterfacePtr map) { map_interface_ = map; }

  void concatenate_lines(const std::vector<LanePtr>& lanes,
                         Line& line_of_corridor);

  bool generate(Point2d point,
                LaneId goal_lane_id,
                double horizon = numeric_double_limits::max());

  Line get_inner_line() const { return current_driving_corridor_.inner; }
  Line get_outer_line() const { return current_driving_corridor_.outer; }
  Line get_center_line() const { return current_driving_corridor_.center; }

 private:
  DrivingCorridor current_driving_corridor_;
  MapInterfacePtr map_interface_;
  LaneId goal_lane_id_;
};

using LocalMapPtr = std::shared_ptr<LocalMap>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_local_map_HPP_
