// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include <vector>
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/road.hpp"
#include "modules/world/map/lane.hpp"
#include "modules/geometry/geometry.hpp"



namespace modules {
namespace world {
namespace map {

using modules::map::Road;
using modules::map::RoadId;
using modules::map::Lane;
using modules::map::LaneId;
using modules::map::Boundary;
using modules::geometry::Line;
using modules::geometry::Polygon;


struct LaneCorridor {
  std::map<float, LanePtr> lanes_;
  Line center_line_;
  Polygon merged_polygon_;
  Boundary left_boundary_;
  Boundary right_boundary_;
};
using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;


struct RoadCorridor {
  RoadPtr GetRoad(RoadId road_id) const {
    return roads_.at(road_id);
  }

  Roads GetRoads() const {
    return roads_;
  }

  Lanes GetLanes(RoadId road_id) const {
    return this->GetRoad(road_id)->GetLanes();
  }

  unsigned int GetHash() const {

  }

  Roads roads_;
  std::vector<LaneCorridorPtr> lane_corridors_;
};
using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif