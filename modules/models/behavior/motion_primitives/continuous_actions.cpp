// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"

namespace modules {
namespace models {
namespace behavior {

BehaviorMotionPrimitives::MotionIdx
BehaviorMPContinuousActions::AddMotionPrimitive(const Input& dynamic_input) {
  motion_primitives_.push_back(dynamic_input);
  return motion_primitives_.size() - 1;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
