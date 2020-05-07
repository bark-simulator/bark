// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RULE_BASED_SIMPLE_BEHAVIOR_HPP_
#define MODULES_MODELS_BEHAVIOR_RULE_BASED_SIMPLE_BEHAVIOR_HPP_

#include <memory>
#include <utility>

#include "modules/models/behavior/rule_based/rule_based.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::Agent;
using modules::world::FrenetPosition;

class BehaviorSimpleRuleBased : public BehaviorRuleBased {
 public:
  explicit BehaviorSimpleRuleBased(const commons::ParamsPtr& params)
    : BehaviorRuleBased(params) {
  }

  virtual ~BehaviorSimpleRuleBased() {}

  std::pair<LaneChangeDecision, world::map::LaneCorridorPtr>
    CheckIfLaneChangeBeneficial(const world::ObservedWorld& observed_world);

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


