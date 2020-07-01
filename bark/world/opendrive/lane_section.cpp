// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <limits>
#include <cmath>
#include <boost/geometry.hpp>
#include "bark/world/opendrive/lane_section.hpp"
#include "bark/world/opendrive/lane_section.hpp"
#include "bark/geometry/commons.hpp"

namespace bark {
namespace world {
namespace opendrive {

void XodrLaneSection::AddLane(const XodrLanePtr& lane)  {
    lanes_[lane->GetId()] = lane;
}

XodrLanePtr XodrLaneSection::GetLaneByPosition(XodrLanePosition pos) {

  XodrLanePtr ret_lane_ptr = nullptr;

  for ( auto const& lane : lanes_ ) {
    if (pos == lane.second->GetLanePosition()) {
      ret_lane_ptr = lane.second;
    }
  } 
  return ret_lane_ptr;
}

}  // namespace opendrive
}  // namespace world
}  // namespace bark
