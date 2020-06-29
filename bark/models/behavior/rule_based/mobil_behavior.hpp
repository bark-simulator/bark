// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_
#define BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_

#include <memory>
#include <map>
#include <utility>
#include <vector>

#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
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


// Mobil lane change model
class BehaviorMobilRuleBased : public BehaviorLaneChangeRuleBased {
 public:
  explicit BehaviorMobilRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorModel(params), 
    BehaviorLaneChangeRuleBased(params) {
    a_thr_ = params->GetReal(
      "BehaviorMobilRuleBased::AThr",
      "Acceleration threshold factor. See Mobil paper.",
      .2);
    politeness_ = params->GetReal(
      "BehaviorMobilRuleBased::Politeness",
      "Politeness factor. See Mobil paper.",
      .5);
  }

  virtual ~BehaviorMobilRuleBased() {}

  virtual std::pair<LaneChangeDecision, LaneCorridorPtr> ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double a_thr_;
  double politeness_;
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
