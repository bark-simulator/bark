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

class BehaviorMotionPrimitives : public BehaviorModel {
 public:
  BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model,
                           commons::Params* params);

  virtual ~BehaviorMotionPrimitives() {}

  virtual Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world);

  typedef unsigned int MotionIdx;
  virtual MotionIdx GetNumMotionPrimitives() const = 0;
  virtual Input GetAction() const = 0;
  void ActionToBehavior(const MotionIdx& motion_idx);

 protected:
  DynamicModelPtr dynamic_model_;
  MotionIdx active_motion_;

  // Parameters
  float integration_time_delta_;
};

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MOTION_PRIMITIVES_HPP_
