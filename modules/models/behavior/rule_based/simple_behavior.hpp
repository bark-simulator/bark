// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RULE_BASED_SIMPLE_BEHAVIOR_HPP_
#define MODULES_MODELS_BEHAVIOR_RULE_BASED_SIMPLE_BEHAVIOR_HPP_

#include <memory>
#include <map>
#include <utility>
#include <vector>

#include "modules/models/behavior/rule_based/rule_based.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::Agent;
using modules::world::AgentPtr;
using modules::world::FrenetPosition;
using modules::world::map::LaneCorridorPtr;
using modules::world::ObservedWorld;
using modules::world::AgentFrenetPair;
using modules::models::dynamic::StateDefinition::VEL_POSITION;

// Agent and LaneCorridor specific
struct AgentInformation {
  // contains ptr and relative distance to ego
  // calculated on other LaneCorridor
  AgentFrenetPair agent_info;
  bool is_vehicle = false;
  double rel_velocity=1e6, rel_distance=1e6;
};

// this is LaneCorridor specific
struct LaneCorridorInformation {
  AgentInformation front;
  AgentInformation rear;
  // should be purely calcualted based using the LaneCorridor
  LaneCorridorPtr lane_corridor;
  double remaining_distance;
};

class BehaviorSimpleRuleBased : public BehaviorRuleBased {
 public:
  explicit BehaviorSimpleRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorRuleBased(params) {
    min_remaining_distance_ = params->GetReal(
      "BehaviorSimpleRuleBased::MinRemainingLaneCorridorDistance",
      "LaneCorridors with less remaning distance are filetered.",
      60.0);
    min_vehicle_rear_distance_ = params->GetReal(
      "BehaviorSimpleRuleBased::MinVehicleRearDistance",
      "Rear vehicle distance.",
      5.0);
    min_vehicle_front_distance_ = params->GetReal(
      "BehaviorSimpleRuleBased::MinVehicleFrontDistance",
      "Front vehicle distance.",
      5.0);
    time_keeping_gap_ = params->GetReal(
      "BehaviorSimpleRuleBased::TimeKeepingGap",
      "Additional time that adds distance based on the rel. vel. to the gap.",
      1.0);
  }

  virtual ~BehaviorSimpleRuleBased() {}

  std::pair<LaneChangeDecision, LaneCorridorPtr>
  CheckIfLaneChangeBeneficial(const ObservedWorld& observed_world) const;

  std::pair<AgentInformation, AgentInformation>
  FrontRearAgents(
    const ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const;

  std::vector<LaneCorridorInformation>
    ScanLaneCorridors(const ObservedWorld& observed_world) const;

  /**
   * @brief Filters std::vector<LaneCorridorInformation> using lambda funcs.
   * 
   * @tparam Func Required for lambda function. In c++ 20 can be removed
   * @param lane_corr_infos Additional LaneCorridor information
   * @param filter Lambda function for filtering
   * @return std::vector<LaneCorridorInformation> Filtered infos
   */
  template<typename Func>
  std::vector<LaneCorridorInformation>
  FilterLaneCorridors(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const Func& filter) const {
    std::vector<LaneCorridorInformation> filtered_lane_corrs;
    std::copy_if(
      lane_corr_infos.begin(),
      lane_corr_infos.end(),
      std::back_inserter(filtered_lane_corrs),
      filter);
    return filtered_lane_corrs;
  }

  LaneCorridorInformation SelectLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const LaneCorridorPtr& lane_corr) const {
    LaneCorridorInformation lc_info;
    for (auto lci : lane_corr_infos) {
      if (lci.lane_corridor == lane_corr) {
        lc_info = lci;
      }
    }
    return lc_info;
  }

  virtual std::pair<LaneChangeDecision, LaneCorridorPtr> ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const;

  double GetVelocity(const AgentPtr& agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  double GetVelocity(
    const std::shared_ptr<const modules::world::objects::Agent> agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double min_remaining_distance_;
  double min_vehicle_rear_distance_;
  double min_vehicle_front_distance_;
  double time_keeping_gap_;
};

inline std::shared_ptr<BehaviorModel> BehaviorSimpleRuleBased::Clone() const {
  std::shared_ptr<BehaviorSimpleRuleBased> model_ptr =
      std::make_shared<BehaviorSimpleRuleBased>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_RULE_BASED_SIMPLE_BEHAVIOR_HPP_


