// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include <vector>
#include "modules/world/opendrive/opendrive.hpp"

using modules::opendrive::XodrRoadPtr;
using modules::opendrive::XodrRoad;
using modules::opendrive::XodrLanes;
using modules::opendrive::XodrLane;

struct Boundary {
  Line line_;
  Type type_;
  // additional parameters: color, ...
};

// ONLY STORE STUFF
struct BarkXodrLane : public XodrLane {
  explicit BarkXodrLane(const XodrLanePtr& lane) : XodrLane(lane) {}

  BarkXodrLanePtr GetLeftXodrLane(unisgned int lane_id) const {
    return left_lanes_.at(lane_id);
  }

  BarkXodrLanePtr GetRightXodrLane(unisgned int lane_id) const {
    return right_lanes_.at(lane_id);
  }

  BarkXodrLanePtr GetNextXodrLane() const {
    return next_lane_id_;
  }

  std::map<int, BarkXodrLanePtr> left_lanes_;  // from_id, to_id
  std::map<int, BarkXodrLanePtr> right_lanes_;  // from_id, to_id
  BarkXodrLanePtr next_lane_;

  Line center_line_;
  Boundary left_boundary_;
  Boundary right_boundary_;
  Polygon polygon_;
};

using BarkXodrLanePtr = std::shared_ptr<BarkXodrLane>;
using BarkXodrLanes = std::map<unsigned int, BarkXodrLanePtr>;

// ONLY STORE STUFF
struct BarkXodrRoad : public XodrRoad {
  explicit BarkXodrRoad(const XodrRoadPtr& road) : XodrRoad(road) {}

  BarkXodrLanes GetXodrLanes() const {
    return bark_lanes_;
  }

  BarkXodrLanePtr GetXodrLane(unisgned int lane_id) const {
    return bark_lanes_.at(lane_id);
  }

  BarkXodrLanePtr GetNextXodrRoad() const {
    return next_road_;
  }

  BarkXodrRoadPtr next_road_;
  BarkXodrLanes bark_lanes_;
};

using BarkXodrRoadPtr = std::shared_ptr<BarkXodrRoad>;
using BarkXodrRoads = std::map<unsigned int, BarkXodrRoadPtr>;


struct XodrLaneCorridor {
  std::map<float, XodrLanePtr> lanes_;  // s, XodrLanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Boundary left_boundary_;
  Boundary right_boundary_;
};

// ONLY STORE STUFF
struct XodrRoadCorridor {
  BarkXodrRoadPtr GetXodrRoad(unsigned int road_id) const {
    return roads_.at(road_id);
  }

  BarkXodrRoads GetXodrRoads() const {
    return roads_;
  }

  BarkXodrLanes GetXodrLanes(unsigned int road_id) const {
    // here we should use a novel lane class
    return this->GetXodrRoad(road_id)->GetXodrLanes();
  }
  

  unsigned int GetHash() const {
    // calculate out of road ids, so creation can be checked
  }

  // Similarily why do we not use a BarkXodrRoad.. merged poly etc
  BarkXodrRoads roads_;
  std::vector<XodrLaneCorridor> lane_corridors_;
};

using XodrRoadCorridorPtr = std::shared_ptr<XodrRoadCorridor>;

#endif