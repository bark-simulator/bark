// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_ROAD_HPP_
#define BARK_WORLD_OPENDRIVE_ROAD_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bark/world/opendrive/commons.hpp"
#include "bark/world/opendrive/lane_section.hpp"
#include "bark/world/opendrive/plan_view.hpp"

namespace bark {
namespace world {
namespace opendrive {

using XodrLaneSections = std::vector<XodrLaneSectionPtr>;

class XodrRoad {
 public:
  XodrRoad(const std::string& name, XodrRoadId id) : id_(id), name_(name) {}

  explicit XodrRoad(const std::shared_ptr<XodrRoad>& road)
      : id_(road->id_),
        name_(road->name_),
        link_(road->link_),
        reference_(road->reference_),
        lane_sections_(road->lane_sections_) {}

  XodrRoad() {}
  virtual ~XodrRoad() {}

  //! getter
  std::shared_ptr<PlanView> GetPlanView() const { return reference_; }
  std::string GetName() const { return name_; }
  XodrRoadId GetId() const { return id_; }
  XodrRoadLink GetLink() const { return link_; }
  XodrLaneSections GetLaneSections() const { return lane_sections_; }

  XodrLanes GetLanes() const {
    XodrLanes lanes;
    for (auto& lane_section : lane_sections_) {
      XodrLanes section_lanes = lane_section->GetLanes();
      lanes.insert(section_lanes.begin(), section_lanes.end());
    }
    return lanes;
  }

  //! setter
  void SetId(XodrRoadId id) { id_ = id; }
  void SetName(const std::string& name) { name_ = name; }
  void SetPlanView(PlanViewPtr p) { reference_ = p; }
  void SetLink(XodrRoadLink l) { link_ = l; }

  void AddLaneSection(XodrLaneSectionPtr l) { lane_sections_.push_back(l); }

 private:
  XodrRoadId id_;
  std::string name_;
  XodrRoadLink link_;
  PlanViewPtr reference_;
  XodrLaneSections lane_sections_;
};

using XodrRoadPtr = std::shared_ptr<XodrRoad>;
using XodrRoads = std::map<XodrRoadId, XodrRoadPtr>;
using XodrRoadSequence = std::vector<XodrRoadId>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_ROAD_HPP_
