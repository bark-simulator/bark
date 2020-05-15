// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_
#define BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_

class PrimitiveConstAcc;
#include "modules/commons/params/params.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acceleration.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_gap_keeping.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;

using modules::models::behavior::primitives::Primitive;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using modules::models::behavior::primitives::PrimitiveConstAcceleration;
using modules::models::behavior::primitives::PrimitiveGapKeeping;
using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;


inline BehaviorMotionPrimitivesPtr BehaviorMacroActionsFromParamServer(const commons::ParamsPtr& params) {
    BehaviorModelPtr ego_prediction_model(
        new BehaviorMPMacroActions(params));

    std::vector<float> acc_vec = params->GetListFloat("AccelerationInputs",
                            "A list of acceleration ", {0, 1, 4, -1, -8});

    std::vector<std::shared_ptr<Primitive>> prim_vec;

    for (auto& acc : acc_vec) {
        auto primitive = std::make_shared<PrimitiveConstAcceleration>(
            params, acc);
        prim_vec.push_back(primitive);
    }

    auto primitive_left = std::make_shared<PrimitiveConstAccChangeToLeft>(
        params);
    prim_vec.push_back(primitive_left);

    auto primitive_right = std::make_shared<PrimitiveConstAccChangeToRight>(
        params);
    prim_vec.push_back(primitive_right);

    auto primitive_gap_keeping = std::make_shared<PrimitiveGapKeeping>(
        params);
    prim_vec.push_back(primitive_gap_keeping);

    for (auto& p : prim_vec) {
        auto idx =
            std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_prediction_model)
                ->AddMotionPrimitive(p);
    }
    return std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model);
}


} // namespace behavior
} // namespace models
} // namespace modules

#endif // BEHAVIOR_MACRO_ACTIONS_FROM_PARAM_SERVER_HPP_