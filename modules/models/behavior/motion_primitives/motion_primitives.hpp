// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_

#include <vector>

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::DynamicModelPtr;
using dynamic::Input;
using dynamic::State;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;

// TODO(@esterle, @bernhard): Add documentation
class BehaviorMotionPrimitives : public BehaviorModel {
 public:
  BehaviorMotionPrimitives(const commons::ParamsPtr& params)
      : BehaviorModel(params),
        active_motion_(),
        integration_time_delta_(params->GetReal(
            "BehaviorMotionPrimitives::IntegrationTimeDelta",
            "the size of the time steps used within the euler integration loop",
            0.02)) {}

  virtual ~BehaviorMotionPrimitives() {}

  // TODO(@hart): use variant
  typedef unsigned int MotionIdx;
  virtual MotionIdx GetNumMotionPrimitives(
      const ObservedWorldPtr& observed_world) = 0;

  void ActionToBehavior(const Action& motion_idx) {
    active_motion_ = motion_idx;
  }

 protected:
  std::vector<Input> motion_primitives_;
  Action active_motion_;

  // Parameters
  float integration_time_delta_;
};

typedef std::shared_ptr<BehaviorMotionPrimitives> BehaviorMotionPrimitivesPtr;

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_
