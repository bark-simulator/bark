// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_OPENDRIVE_HPP_
#define MODULES_WORLD_OPENDRIVE_OPENDRIVE_HPP_

#include <vector>
#include <map>

#include "modules/world/opendrive/road.hpp"
#include "modules/world/opendrive/junction.hpp"
#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

using Junctions = std::map<uint32_t, std::shared_ptr<Junction>>;

class OpenDriveMap {
 public:
  OpenDriveMap() : roads_(), lanes_(), junctions_() {}
  ~OpenDriveMap() {}

  //! setter functions
  bool add_road(std::shared_ptr<XodrRoad> r) {
    roads_[r->GetId()] = r;
    XodrLanes road_lanes = r->GetLanes();
    lanes_.insert(road_lanes.begin(), road_lanes.end());
    return true;
  }
  bool add_junction(std::shared_ptr<Junction> j) {
    junctions_[j->GetId()] = j;
    return true;
  }
  //! getter functions
  XodrRoadPtr get_road(XodrRoadId id) const { return roads_.at(id); }
  std::shared_ptr<Junction> get_junction(uint32_t id) const {
    return junctions_.at(id);
  }
  XodrLanePtr GetLane(XodrLaneId id) const { return lanes_.at(id); }

  XodrRoads get_roads() const { return roads_; }
  Junctions get_junctions() const { return junctions_; }
  XodrLanes GetLanes() const { return lanes_;}

  std::pair<modules::geometry::Point2d, modules::geometry::Point2d> bounding_box() const {
    modules::geometry::Line all_lanes_linestrings;
    for (auto &road : get_roads()) {
      for (auto &lane_section : road.second->GetLaneSections()) {
        for (auto &lane : lane_section->GetLanes()) {
          auto linestring = lane.second->get_line();
          all_lanes_linestrings.append_linestring(linestring);
        }
      }
    }
    return all_lanes_linestrings.bounding_box();
  }

 private:
  XodrRoads roads_;
  XodrLanes lanes_;
  Junctions junctions_;
};

using OpenDriveMapPtr = std::shared_ptr<OpenDriveMap>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_OPENDRIVE_HPP_
