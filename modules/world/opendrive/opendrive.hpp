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

class OpenDriveMap {  // TODO(@hart): rename to OpenDrive.. OpenDriveMap is too general
 public:
  OpenDriveMap() : roads_(), lanes_(), junctions_() {}
  ~OpenDriveMap() {}

  //! setter functions
  bool add_road(std::shared_ptr<Road> r) {
    roads_[r->get_id()] = r;
    Lanes road_lanes = r->get_lanes();
    lanes_.insert(road_lanes.begin(), road_lanes.end());
    return true;
  }
  bool add_junction(std::shared_ptr<Junction> j) {
    junctions_[j->get_id()] = j;
    return true;
  }
  //! getter functions
  RoadPtr get_road(RoadId id) const { return roads_.at(id); }
  std::shared_ptr<Junction> get_junction(uint32_t id) const {
    return junctions_.at(id);
  }
  LanePtr get_lane(LaneId id) const { return lanes_.at(id); }

  Roads get_roads() const { return roads_; }
  Junctions get_junctions() const { return junctions_; }
  Lanes get_lanes() const { return lanes_;}

  std::pair<modules::geometry::Point2d, modules::geometry::Point2d> bounding_box() const {
    modules::geometry::Line all_lanes_linestrings;
    for (auto &road : get_roads()) {
      for (auto &lane_section : road.second->get_lane_sections()) {
        for (auto &lane : lane_section->get_lanes()) {
          auto linestring = lane.second->get_line();
          all_lanes_linestrings.append_linestring(linestring);
        }
      }
    }
    return all_lanes_linestrings.bounding_box();
  }

 private:
  Roads roads_;
  Lanes lanes_;
  Junctions junctions_;
};

typedef std::shared_ptr<OpenDriveMap> OpenDriveMapPtr;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_OPENDRIVE_HPP_
