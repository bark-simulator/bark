// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_
#define BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::StateDefinition::VEL_POSITION;
using bark::world::Agent;
using bark::world::AgentFrenetPair;
using bark::world::AgentPtr;
using bark::world::FrenetPosition;
using bark::world::ObservedWorld;
using bark::world::map::LaneCorridorPtr;

// Mobil lane change model
class BehaviorMobilRuleBased : public BehaviorLaneChangeRuleBased {
 public:
  explicit BehaviorMobilRuleBased(const commons::ParamsPtr& params)
      : BehaviorModel(params), BehaviorLaneChangeRuleBased(params) {
    a_thr_ =
        params->GetReal("BehaviorMobilRuleBased::AThr",
                        "Acceleration threshold factor. See Mobil paper.", .2);
    politeness_ = params->GetReal("BehaviorMobilRuleBased::Politeness",
                                  "Politeness factor. See Mobil paper.", .5);
    safe_deceleration_ = params->GetReal(
        "BehaviorMobilRuleBased::SafeDeceleration",
        "Maximum deceleration for follower in target lane.", 4.0);
    assert(safe_deceleration_ >= 0);
  }

  virtual ~BehaviorMobilRuleBased() {}

  double CalcLongRawAccWithoutLeader(const world::LaneCorridorPtr& lane_corr,
                                     const bark::geometry::Point2d& pos,
                                     double vel) const;
  virtual std::pair<LaneChangeDecision, LaneCorridorPtr> ChooseLaneCorridor(
      const std::vector<LaneCorridorInformation>& lane_corr_infos,
      const ObservedWorld& observed_world) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double a_thr_;
  double politeness_;
  double safe_deceleration_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMobilRuleBased::Clone() const {
  std::shared_ptr<BehaviorMobilRuleBased> model_ptr =
      std::make_shared<BehaviorMobilRuleBased>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_
