// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include <vector>
#include "modules/world/opendrive/opendrive.hpp"

using modules::opendrive::RoadPtr;
using modules::opendrive::Road;
using modules::opendrive::Lanes;
using modules::opendrive::Lane;

struct Boundary {
  Line line_;
  Type type_;
  // additional parameters: color, ...
};

// ONLY STORE STUFF
struct BarkLane : public Lane {
  explicit BarkLane(const LanePtr& lane) : Lane(lane) {}

  BarkLanePtr GetLeftLane(unisgned int lane_id) const {
    return left_lanes_.at(lane_id);
  }

  BarkLanePtr GetRightLane(unisgned int lane_id) const {
    return right_lanes_.at(lane_id);
  }

  BarkLanePtr GetNextLane() const {
    return next_lane_id_;
  }

  std::map<int, BarkLanePtr> left_lanes_;  // from_id, to_id
  std::map<int, BarkLanePtr> right_lanes_;  // from_id, to_id
  BarkLanePtr next_lane_;

  Line center_line_;
  Boundary left_boundary_;
  Boundary right_boundary_;
  Polygon polygon_;
};

using BarkLanePtr = std::shared_ptr<BarkLane>;
using BarkLanes = std::map<unsigned int, BarkLanePtr>;

// ONLY STORE STUFF
struct BarkRoad : public Road {
  explicit BarkRoad(const RoadPtr& road) : Road(road) {}

  BarkLanes GetLanes() const {
    return bark_lanes_;
  }

  BarkLanePtr GetLane(unisgned int lane_id) const {
    return bark_lanes_.at(lane_id);
  }

  BarkLanePtr GetNextRoad() const {
    return next_road_;
  }

  BarkRoadPtr next_road_;
  BarkLanes bark_lanes_;
};

using BarkRoadPtr = std::shared_ptr<BarkRoad>;
using BarkRoads = std::map<unsigned int, BarkRoadPtr>;


struct LaneCorridor {
  std::map<float, LanePtr> lanes_;  // s, LanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Boundary left_boundary_;
  Boundary right_boundary_;
};

// ONLY STORE STUFF
struct RoadCorridor {
  BarkRoadPtr GetRoad(unsigned int road_id) const {
    return roads_.at(road_id);
  }

  BarkRoads GetRoads() const {
    return roads_;
  }

  BarkLanes GetLanes(unsigned int road_id) const {
    // here we should use a novel lane class
    return this->GetRoad(road_id)->GetLanes();
  }
  

  unsigned int GetHash() const {
    // calculate out of road ids, so creation can be checked
  }

  // Similarily why do we not use a BarkRoad.. merged poly etc
  BarkRoads roads_;
  std::vector<LaneCorridor> lane_corridors_;
};

using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;

#endif