// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_ROAD_HPP_
#define MODULES_WORLD_OPENDRIVE_ROAD_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace opendrive {

using LaneSections = std::vector<LaneSectionPtr>;

class Road {
 public:
  Road(const std::string& name, RoadId id) : id_(id), name_(name) {}
  Road() {}
  virtual ~Road() {}

  //! getter
  std::shared_ptr<PlanView> get_plan_view() const { return reference_; }
  std::string get_name() const { return name_; }
  RoadId get_id() const { return id_; }
  RoadLink get_link() const { return link_; }
  LaneSections get_lane_sections() const { return lane_sections_; }

  // TODO (@hart): implement function get_next_roads()
  // either one road from successor or multiple roads from junction

  Lanes get_lanes() const {
    Lanes lanes;
    for (auto& lane_section : lane_sections_) {
      Lanes section_lanes = lane_section->get_lanes();
      lanes.insert(section_lanes.begin(), section_lanes.end());
    }
    return lanes;
  }


  //! setter
  void set_id(RoadId id) { id_ = id; }
  void set_name(const std::string& name) { name_ = name; }
  void set_plan_view(PlanViewPtr p) { reference_ = p; }
  void set_link(RoadLink l) { link_ = l; }

  void add_lane_section(LaneSectionPtr l) {
    // additionally we need lane 0 in LaneSection
    // Lane lane_0 = create_lane();
    lane_sections_.push_back(l);
  }

 private:
  RoadId id_;
  std::string name_;
  RoadLink link_;
  PlanViewPtr reference_;
  LaneSections lane_sections_;
};

using RoadPtr = std::shared_ptr<Road>;
using Roads = std::map<RoadId, RoadPtr>;
using RoadSequence = std::vector<RoadId>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_ROAD_HPP_
