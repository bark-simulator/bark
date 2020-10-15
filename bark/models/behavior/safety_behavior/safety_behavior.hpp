// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_SAFETY_BEHAVIOR_HPP_
#define BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_SAFETY_BEHAVIOR_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;

class BehaviorSafety : public BehaviorModel {
 public:safety_behavior
  explicit BehaviorSafety(const commons::ParamsPtr& params)
    : BehaviorModel(params) {}

  virtual ~BehaviorSafety() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::shared_ptr<BehaviorModel> sub_behavior_model_;
};

inline std::shared_ptr<BehaviorModel> BehaviorSafety::Clone() const {
  std::shared_ptr<BehaviorSafety> model_ptr =
      std::make_shared<BehaviorSafety>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_SAFETY_BEHAVIOR_HPP_
