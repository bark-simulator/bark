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
                      center(Line()),
                      computed(false) {}
  //! getter
  Line get_outer() { return outer; }
  Line get_inner() { return inner; }
  Line get_center() { return center; }

  //! getter
  void set_outer(const Line& o) { outer = o; }
  void set_inner(const Line& o) { inner = o; }
  void set_center(const Line& o) { center = o; }

  Line outer, inner, center;
  std::vector< std::pair<int, LaneId> > lane_ids_; //1st entry is the index from where the 2nd value lane id is valid
  bool computed;
  // TODO(@fortiss): what IF functions would we like here
};

class LocalMap {
 public:
  explicit LocalMap(LaneId goal_lane_id,
                    const MapInterfacePtr& map_interface) :
    driving_corridor_(DrivingCorridor()),
    horizon_driving_corridor_(DrivingCorridor()),
    map_interface_(map_interface),
    goal_lane_id_(goal_lane_id) {}

  LocalMap(const LocalMap& lm) :
    driving_corridor_(lm.driving_corridor_),
    horizon_driving_corridor_(lm.horizon_driving_corridor_),
    map_interface_(lm.map_interface_),
    goal_lane_id_(lm.goal_lane_id_) {}

  void set_goal_lane_id(LaneId goal_lane_id) { goal_lane_id_ = goal_lane_id_; }

  void set_map_interface(MapInterfacePtr map) { map_interface_ = map; }

  void concatenate_lines(const std::vector<LanePtr>& lanes,
                         Line& line_of_corridor,
                         std::vector< std::pair<int, LaneId> >& lane_ids);

  bool generate(Point2d point,
                LaneId goal_lane_id,
                double horizon = numeric_double_limits::max());

  Line line_horizon(const Line& line,
                    const Point2d& p,
                    double horizon);

  bool compute_horizon_corridor(const Point2d& p, double horizon);

  DrivingCorridor get_driving_corridor() const {
    return driving_corridor_;
  }

  DrivingCorridor get_horizon_driving_corridor() const {
    return horizon_driving_corridor_;
  }

  bool has_generated_driving_corridor() {
    return driving_corridor_.computed;
  }

 private:
  DrivingCorridor driving_corridor_;
  DrivingCorridor horizon_driving_corridor_;
  MapInterfacePtr map_interface_;
  LaneId goal_lane_id_;
};

using LocalMapPtr = std::shared_ptr<LocalMap>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_local_map_HPP_
