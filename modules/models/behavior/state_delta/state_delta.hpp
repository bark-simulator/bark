// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_STATE_DELTA_STATE_DELTA_HPP_
#define MODULES_MODELS_BEHAVIOR_STATE_DELTA_STATE_DELTA_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using dynamic::State;
using world::objects::AgentId;
using world::ObservedWorld;

class BehaviorStateDelta : public BehaviorModel {
 public:
  explicit BehaviorStateDelta(const State& state_delta, commons::Params *params) :
    BehaviorModel(params),
    state_delta_(state_delta) {}

  virtual ~BehaviorStateDelta() {}

  Trajectory Plan(float delta_time,
                 const ObservedWorld& observed_world);

  virtual BehaviorModel *Clone() const;
private:
  State state_delta_;
};

inline BehaviorModel *BehaviorStateDelta::Clone() const {
  return new BehaviorStateDelta(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_
