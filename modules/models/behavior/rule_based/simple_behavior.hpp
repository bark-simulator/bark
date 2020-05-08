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
  double rel_velocity, rel_distance;
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
  }

  virtual ~BehaviorSimpleRuleBased() {}

  // check whether a lane change should be performed
  std::pair<LaneChangeDecision, LaneCorridorPtr>
  CheckIfLaneChangeBeneficial(const ObservedWorld& observed_world) const;

  // return neat information for front/rear agent
  std::pair<AgentInformation, AgentInformation>
  FrontRearAgents(
    const ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const;

  // check neighboring LaneCorridors (left/right)
  // include remaining distance
  std::vector<LaneCorridorInformation>
  ScanLaneCorridors(const ObservedWorld& observed_world) const;

  // filter lane corridors using lambdas
  template<typename Func> 
  std::vector<LaneCorridorInformation>
  FilterLaneCorridors(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const Func&) const {
    std::vector<LaneCorridorInformation> filtered_lane_corrs;
    std::copy_if(
      lane_corr_infos.begin(),
      lane_corr_infos.end(),
      std::back_inserter(filtered_lane_corrs),
      [](LaneCorridorInformation li){return li.remaining_distance>=50;});
    return filtered_lane_corrs;
  }

  double GetVelocity(const AgentPtr& agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  // TODO(@all): making the agent const is a good idea, but needs to be unified
  double GetVelocity(
    const std::shared_ptr<const modules::world::objects::Agent> agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  virtual std::shared_ptr<BehaviorModel> Clone() const;
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


