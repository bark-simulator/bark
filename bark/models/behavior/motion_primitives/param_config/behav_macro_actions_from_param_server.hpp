// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_
#define BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_

class PrimitiveConstAcc;
#include "bark/commons/params/params.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_stay_lane.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_gap_keeping.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::prediction::PredictionSettings;

using bark::models::behavior::primitives::Primitive;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using bark::models::behavior::primitives::PrimitiveConstAccStayLane;
using bark::models::behavior::primitives::PrimitiveGapKeeping;
using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::prediction::PredictionSettings;

inline BehaviorMotionPrimitivesPtr BehaviorMacroActionsFromParamServer(
    const commons::ParamsPtr& params) {
  BehaviorModelPtr ego_prediction_model(new BehaviorMPMacroActions(params));

  std::vector<float> acc_vec = params->GetListFloat(
      "AccelerationInputs", "A list of acceleration ", {0, 1, 4, -1, -8});

  std::vector<std::shared_ptr<Primitive>> prim_vec;

  for (auto& acc : acc_vec) {
    auto primitive = std::make_shared<PrimitiveConstAccStayLane>(params, acc);
    prim_vec.push_back(primitive);
  }

  auto primitive_left = std::make_shared<PrimitiveConstAccChangeToLeft>(params);
  prim_vec.push_back(primitive_left);

  auto primitive_right =
      std::make_shared<PrimitiveConstAccChangeToRight>(params);
  prim_vec.push_back(primitive_right);

  auto primitive_gap_keeping = std::make_shared<PrimitiveGapKeeping>(params);
  prim_vec.push_back(primitive_gap_keeping);

  for (auto& p : prim_vec) {
    auto idx =
        std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_prediction_model)
            ->AddMotionPrimitive(p);
  }
  return std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
      ego_prediction_model);
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_