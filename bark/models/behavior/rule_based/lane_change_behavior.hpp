// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RULE_BASED_LANE_CHANGE_BEHAVIOR_HPP_
#define BARK_MODELS_BEHAVIOR_RULE_BASED_LANE_CHANGE_BEHAVIOR_HPP_

#include <memory>
#include <map>
#include <utility>
#include <vector>

#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::Agent;
using bark::world::AgentPtr;
using bark::world::FrenetPosition;
using bark::world::map::LaneCorridorPtr;
using bark::world::ObservedWorld;
using bark::world::AgentFrenetPair;
using bark::models::dynamic::StateDefinition::VEL_POSITION;

enum LaneChangeDecision {
  KeepLane = 0,
  ChangeLeft = 1,
  ChangeRight = 2,
  Undefined = 3,
  ChangeLane = 4
};

enum RuleBasedState {
  Idle = 0,
  IsChanging = 1
};

// Agent and LaneCorridor specific
struct AgentInformation {
  // contains ptr and relative distance to ego
  // calculated on other LaneCorridor
  AgentFrenetPair agent_info;
  bool is_vehicle = false;
  double rel_velocity = 1e6, rel_distance = 1e6;
};

// this is LaneCorridor specific
struct LaneCorridorInformation {
  AgentInformation front;
  AgentInformation rear;
  // should be purely calcualted based using the LaneCorridor
  LaneCorridorPtr lane_corridor;
  double remaining_distance;
};


// Behavior that changes lanes if there is more free-space
class BehaviorLaneChangeRuleBased : public BehaviorIDMLaneTracking {
 public:
  explicit BehaviorLaneChangeRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorIDMLaneTracking(params) {
    min_remaining_distance_ = params->GetReal(
      "BehaviorLaneChangeRuleBased::MinRemainingLaneCorridorDistance",
      "LaneCorridors with less remaning distance are filetered.",
      60.0);
    min_vehicle_rear_distance_ = params->GetReal(
      "BehaviorLaneChangeRuleBased::MinVehicleRearDistance",
      "Rear vehicle distance.",
      5.0);
    min_vehicle_front_distance_ = params->GetReal(
      "BehaviorLaneChangeRuleBased::MinVehicleFrontDistance",
      "Front vehicle distance.",
      5.0);
    time_keeping_gap_ = params->GetReal(
      "BehaviorLaneChangeRuleBased::TimeKeepingGap",
      "Additional time that adds distance based on the rel. vel. to the gap.",
      1.0);
  }

  virtual ~BehaviorLaneChangeRuleBased() {}

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

  std::pair<LaneCorridorInformation, bool> SelectLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const LaneCorridorPtr& lane_corr) const {
    LaneCorridorInformation lc_info;
    bool has_info = false;
    for (auto lci : lane_corr_infos) {
      if (lci.lane_corridor == lane_corr) {
        lc_info = lci;
        has_info = true;
      }
    }
    return std::pair<LaneCorridorInformation, bool>(lc_info, has_info);
  }

  virtual std::pair<LaneChangeDecision, LaneCorridorPtr> ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const;

  double GetVelocity(const AgentPtr& agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  double GetVelocity(
    const std::shared_ptr<const bark::world::objects::Agent> agent) const {
    const auto& state = agent->GetCurrentState();
    return state[VEL_POSITION];
  }

  virtual Trajectory Plan(
    float delta_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double min_remaining_distance_;
  double min_vehicle_rear_distance_;
  double min_vehicle_front_distance_;
  double time_keeping_gap_;
};

inline std::shared_ptr<BehaviorModel> BehaviorLaneChangeRuleBased::Clone() const {
  std::shared_ptr<BehaviorLaneChangeRuleBased> model_ptr =
      std::make_shared<BehaviorLaneChangeRuleBased>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RULE_BASED_LANE_CHANGE_BEHAVIOR_HPP_


