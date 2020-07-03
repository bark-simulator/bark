// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_HELPERS_HPP_
#define BARK_WORLD_OPENDRIVE_HELPERS_HPP_

#include "bark/world/opendrive/lane.hpp"

namespace bark {
namespace world {
namespace opendrive {

XodrLaneSequence IntersectBasedOnIds(const XodrLaneSequence& lane_sequence,
                                     const XodrLanes& lanes) {
  XodrLaneSequence intersect;
  for (auto& lane_id_1 : lane_sequence) {
    for (auto& lane2 : lanes) {
      if (lane_id_1 == lane2.second->GetId()) {
        intersect.push_back(lane_id_1);
      }
    }
  }
  return intersect;
}

bool ConcatenateLanes(const XodrLane& lane1, XodrLane& concatenated_lane) {
  return false;
}

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif