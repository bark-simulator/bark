// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include "modules/world/opendrive/opendrive.hpp"

using modules::opendrive::RoadPtr;
using modules::opendrive::Road;
using modules::opendrive::Lanes;
using modules::opendrive::Lane;

struct BarkLane : public Lane {
  explicit BarkLane(const LanePtr& lane) : Lane(road) {}
  // polygon_
  // center_
  // left and right boundary_
  // ...
};

using BarkLanePtr = std::shared_ptr<BarkLane>;
using BarkLanes = std::map<unsigned int, BarkRoadPtr>;

struct BarkRoad : public Road {
  explicit BarkRoad(const RoadPtr& road) : Road(road) {}

  BarkLanes GetLanes() const {
    return bark_lanes_;
  }

  BarkLanePtr GetLane(unisgned int lane_id) const {
    return bark_lanes_.at(lane_id);
  }

  BarkLanePtr GetSuccesorLane(unisgned int lane_id) const {
    // we need the roadgraph for this
  }

  BarkLanePtr GetPredecessorLane(unisgned int lane_id) const {
    // we need the roadgraph for this
  }

  BarkLanePtr GetLeftLane(unisgned int lane_id) const {
    // we need the roadgraph for this
  }

  BarkLanePtr GetRightLane(unisgned int lane_id) const {
    // we need the roadgraph for this
  }

  BarkLanes bark_lanes_;
};
using BarkRoadPtr = std::shared_ptr<BarkRoad>;
using BarkRoads = std::map<unsigned int, BarkRoadPtr>;


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
};

using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;

#endif