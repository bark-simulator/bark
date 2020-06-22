// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/evaluator_safe_distance.hpp"

#include "bark/world/evaluation/ltl/labels/safe_distance_label_function.hpp"

namespace bark {
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
}  // namespace bark
