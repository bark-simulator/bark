// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_
#define MODULES_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_

#include <memory>
#include <map>
#include <utility>
#include <vector>

#include "modules/models/behavior/rule_based/base_rule_based.hpp"
#include "modules/models/behavior/rule_based/simple_behavior.hpp"
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


class BehaviorMobilRuleBased : public BehaviorSimpleRuleBased {
 public:
  explicit BehaviorMobilRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorSimpleRuleBased(params) {
    a_thr_ = params->GetReal(
      "BehaviorMobilRuleBased::AThr",
      "a_thr.",
      .2);
    politeness_ = params->GetReal(
      "BehaviorMobilRuleBased::Politeness",
      "Politeness factor.",
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
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_RULE_BASED_MOBIL_BEHAVIOR_HPP_
