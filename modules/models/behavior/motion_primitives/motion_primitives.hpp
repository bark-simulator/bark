// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_STATE_DELTA_STATE_DELTA_HPP_
#define MODULES_MODELS_BEHAVIOR_STATE_DELTA_STATE_DELTA_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using dynamic::State;
using dynamic::DynamicModelPtr;
using dynamic::Input;
using world::objects::AgentId;
using world::ObservedWorld;

class BehaviorMotionPrimitives : public BehaviorModel {
 public:
  BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model, commons::Params *params);

  virtual ~BehaviorMotionPrimitives() {}

  virtual Trajectory Plan(float delta_time,
                 const ObservedWorld& observed_world);

  typedef unsigned int MotionIdx;
  MotionIdx AddMotionPrimitive(const Input& dynamic_input, const float time_span);
  void ActionToBehavior(const MotionIdx& motion_idx);

  virtual BehaviorModel *Clone() const;
private:
  DynamicModelPtr dynamic_model_;
  std::vector<Trajectory> motion_primitives_; 
  MotionIdx active_motion_;

  // Parameters
  float integration_time_delta_;
};

inline BehaviorModel *BehaviorMotionPrimitives::Clone() const {
  return new BehaviorMotionPrimitives(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_
