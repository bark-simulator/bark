// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_LOCAL_MAP_HPP_
#define MODULES_WORLD_MAP_LOCAL_MAP_HPP_

#include <vector>
#include <string>
#include <ostream>
#include <limits>
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/driving_corridor.hpp"
#include "modules/geometry/geometry.hpp"
#include "modules/world/map/frenet.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LaneId;
using modules::world::opendrive::LanePtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPolygon;
using numeric_double_limits = std::numeric_limits<double>;
using models::dynamic::State;
using models::dynamic::StateDefinition;
using namespace modules::geometry;


class LocalMap {
 public:
  LocalMap(const GoalDefinitionPtr& goal_definition,
                    const MapInterfacePtr& map_interface) :
    driving_corridor_(DrivingCorridor()),
    horizon_driving_corridor_(DrivingCorridor()),
    map_interface_(map_interface),
    goal_definition_(goal_definition),
    goal_lane_id_(LaneId()) {}

  LocalMap(LaneId goal_lane_id, const GoalDefinitionPtr& goal_definition,
                    const DrivingCorridor& driving_corridor) :
    driving_corridor_(driving_corridor),
    goal_definition_(goal_definition),
    goal_lane_id_(goal_lane_id) {}
 
  LocalMap(const LocalMap& lm) :
    driving_corridor_(lm.driving_corridor_),
    horizon_driving_corridor_(lm.horizon_driving_corridor_),
    map_interface_(lm.map_interface_),
    goal_definition_(lm.goal_definition_),
    goal_lane_id_(lm.goal_lane_id_) {}

  //! Setter
  void set_goal_definition(GoalDefinitionPtr goal_definition) {
    goal_definition_ = goal_definition;
  }
  void set_goal_lane_id(LaneId goal_lane_id) { goal_lane_id_ = goal_lane_id_; }
  void set_map_interface(MapInterfacePtr map) { map_interface_ = map; }

  //! Getter
  DrivingCorridor get_driving_corridor() const {
    return driving_corridor_;
  }
  DrivingCorridor get_horizon_driving_corridor() const {
    return horizon_driving_corridor_;
  }
  bool has_generated_driving_corridor() {
    return driving_corridor_.computed;
  }

  LaneId get_goal_lane_id() const {
    return goal_lane_id_;
  }

  GoalDefinitionPtr get_goal_definition() const {
    return goal_definition_;
  }

  //! Functions
  void ConcatenateLines(const std::vector<LanePtr>& lanes,
                        Line& line_of_corridor,
                        std::vector<std::pair<int, LaneId>>& lane_ids);

  LaneId GoalLaneIdFromPolygon(const modules::geometry::Polygon& goal_polygon);

  LanePtr FindLane(const Point2d& point) const;

  bool HasCorrectDrivingDirection(const State& state) const;

  bool Generate(Point2d point);
  bool RecalculateDrivingCorridor(const Point2d &point);
  Line CalculateLineHorizon(const Line& line, const Point2d& p, double horizon);

  bool ComputeHorizonCorridor(const Point2d& p, double horizon);

  // TODO: extend this by a good data structure for meging/ splitting
  std::vector<DrivingCorridorPtr> get_left_adjacent() const;
  // TODO: extend this by a good data structure for meging/ splitting
  std::vector<DrivingCorridorPtr> get_right_adjacent() const;
  virtual LocalMap *Clone() const;

 private:
  DrivingCorridor driving_corridor_;
  DrivingCorridor horizon_driving_corridor_;
  MapInterfacePtr map_interface_;
  GoalDefinitionPtr goal_definition_;
  LaneId goal_lane_id_;
};

using LocalMapPtr = std::shared_ptr<LocalMap>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_LOCAL_MAP_HPP_
