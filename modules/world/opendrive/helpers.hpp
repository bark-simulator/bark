// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_HELPERS_HPP_
#define MODULES_WORLD_OPENDRIVE_HELPERS_HPP_

#include "modules/world/opendrive/lane.hpp"


namespace modules {
namespace world {
namespace opendrive {

LaneSequence intersect_based_on_ids(const LaneSequence& lane_sequence, const Lanes& lanes)
{
  LaneSequence intersect;
  for (auto& lane_id_1: lane_sequence ) {
    for (auto& lane2: lanes) {
      if (lane_id_1 == lane2.second->get_id())
      {
        intersect.push_back(lane_id_1);
      }
    }
  }
  return intersect;
}


bool concatenate_lanes(const Lane& lane1, Lane& concatenated_lane)
{
  return false;
}








}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif