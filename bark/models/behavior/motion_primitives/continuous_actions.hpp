// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace behavior {

// TODO(@esterle, @bernhard): Add documentationx
class BehaviorMPContinuousActions : public BehaviorMotionPrimitives {
 public:
  BehaviorMPContinuousActions(const commons::ParamsPtr& params)
      : BehaviorMotionPrimitives(params), motion_primitives_() {}

  virtual ~BehaviorMPContinuousActions() {}

  virtual Trajectory Plan(float min_planning_time,
                          const ObservedWorld& observed_world);

  virtual MotionIdx GetNumMotionPrimitives(
      const ObservedWorldPtr& observed_world) {
    return motion_primitives_.size();
  }
  virtual Input GetAction() const {
    return motion_primitives_[boost::get<DiscreteAction>(active_motion_)];
  }
  MotionIdx AddMotionPrimitive(const Input& dynamic_input);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::vector<Input> motion_primitives_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMPContinuousActions::Clone()
    const {
  std::shared_ptr<BehaviorMPContinuousActions> model_ptr =
      std::make_shared<BehaviorMPContinuousActions>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
