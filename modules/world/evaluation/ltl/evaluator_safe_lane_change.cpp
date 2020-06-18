// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/ltl/evaluator_safe_lane_change.hpp"

#include "modules/world/evaluation/labels/lane_change_label_function.hpp"
#include "modules/world/evaluation/labels/safe_distance_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {

/// Keep a safe distance to the successor vehicle when changing lanes!
const char EvaluatorSafeLaneChange::formula_[] = "G (lane_change -> sd_rear)";

const LabelFunctions EvaluatorSafeLaneChange::labels_ = {
    LabelFunctionPtr(new SafeDistanceLabelFunction(
        "sd_rear", true, reaction_time, decel_ego, decel_rear)),
    LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"))};

const char EvaluatorSafeLaneChangeAssumption::formula_[] = "G !lane_change";

const LabelFunctions EvaluatorSafeLaneChangeAssumption::labels_ = {
    LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
