// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_LANE_SECTION_HPP_
#define BARK_WORLD_OPENDRIVE_LANE_SECTION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bark/world/opendrive/lane.hpp"

namespace bark {
namespace world {
namespace opendrive {

class XodrLaneSection {
 public:
  explicit XodrLaneSection(float s) : s_(s) {}
  ~XodrLaneSection() {}

  XodrLanes GetLanes() const { return lanes_; }

  XodrLanePtr GetLaneByPosition(XodrLanePosition pos);

  //! setter functions
  void AddLane(const XodrLanePtr& l);

  //! getter functions
  float GetS() const { return s_; }

 private:
  float s_;
  XodrLanes lanes_;
};

inline std::string print(const XodrLaneSection& ls) {
  std::stringstream ss;
  ss << "s: " << ls.GetS() << std::endl;
  for (auto const& l : ls.GetLanes())
    ss << "XodrLane: " << print(*(l.second)) << std::endl;
  return ss.str();
}

using XodrLaneSectionPtr = std::shared_ptr<XodrLaneSection>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_LANE_SECTION_HPP_
