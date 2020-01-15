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


struct LaneCorridor {
  std::map<float, LanePtr> lanes_;  // s, XodrLanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Boundary left_boundary_;
  Boundary right_boundary_;
};


// ONLY STORE STUFF
struct RoadCorridor {
  RoadPtr GetRoad(unsigned int road_id) const {
    return roads_.at(road_id);
  }

  Roads GetRoads() const {
    return roads_;
  }

  Lanes GetLanes(unsigned int road_id) const {
    // here we should use a novel lane class
    return this->GetRoad(road_id)->GetLanes();
  }

  unsigned int GetHash() const {
    // calculate out of road ids, so creation can be checked
  }

  // Similarily why do we not use a Road.. merged poly etc
  Roads roads_;
  std::vector<LaneCorridor> lane_corridors_;
};

using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;

#endif