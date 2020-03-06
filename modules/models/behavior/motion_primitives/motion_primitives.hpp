// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_

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

class BehaviorMotionPrimitives : public BehaviorModel {
 public:
  BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model,
                           const commons::ParamsPtr& params)
      : BehaviorModel(params),
        dynamic_model_(dynamic_model),
        active_motion_(),
        integration_time_delta_(params->GetReal(
            "BehaviorMotionPrimitives::IntegrationTimeDelta",
            "the size of the time steps used within the euler integration loop",
            0.02)) {}

  virtual ~BehaviorMotionPrimitives() {}

  typedef unsigned int MotionIdx;
  virtual MotionIdx GetNumMotionPrimitives(const ObservedWorldPtr& observed_world) const = 0;
  //virtual Input GetAction() const = 0;
  void ActionToBehavior(const MotionIdx& motion_idx) {
    active_motion_ = motion_idx;
  }

  // virtual std::shared_ptr<BehaviorModel> Clone() const;

 protected:
  DynamicModelPtr dynamic_model_;
  std::vector<Input> motion_primitives_;
  MotionIdx active_motion_;

  // Parameters
  float integration_time_delta_;
};

// inline std::shared_ptr<BehaviorModel> BehaviorMotionPrimitives::Clone() const {
//   std::shared_ptr<BehaviorMotionPrimitives> model_ptr =
//     std::make_shared<BehaviorMotionPrimitives>(*this);
//   return model_ptr;
// }

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_
