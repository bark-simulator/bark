// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_safe_distance.hpp"

#include "modules/world/evaluation/labels/safe_distance_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {

/// Always keep a safe distance to the front vehicle!
const char EvaluatorSafeDistance::formula_[] = "G sd_front";

const LabelFunctions EvaluatorSafeDistance::labels_ = {
    LabelFunctionPtr(new SafeDistanceLabelFunction(
        "sd_front", false, EvaluatorSafeDistance::reaction_time,
        EvaluatorSafeDistance::decel_ego, EvaluatorSafeDistance::decel_front))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
