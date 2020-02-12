// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorMPContinousActions : public BehaviorMotionPrimitives {
 public:
  BehaviorMPContinousActions(const DynamicModelPtr& dynamic_model,
                             commons::Params* params)
      : BehaviorMotionPrimitives(dynamic_model, params), motion_primitives_() {}

  virtual ~BehaviorMPContinousActions() {}

  virtual MotionIdx GetNumMotionPrimitives() const {
    return motion_primitives_.size();
  }
  virtual Input GetAction() const { return motion_primitives_[active_motion_]; }
  MotionIdx AddMotionPrimitive(const Input& dynamic_input);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::vector<Input> motion_primitives_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMPContinousActions::Clone()
    const {
  std::shared_ptr<BehaviorMPContinousActions> model_ptr =
      std::make_shared<BehaviorMPContinousActions>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
