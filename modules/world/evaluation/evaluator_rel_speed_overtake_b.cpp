// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_rel_speed_overtake_b.hpp"

#include "modules/world/evaluation/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/labels/front_of_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/rel_speed_label_function.hpp"
#include "modules/world/evaluation/labels/right_of_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorRelSpeedOvertakeB::formula_[] =
    "G (behind_j & X(behind_j U left_j U in_front_j) -> (speed_diff_j U "
    "in_front_j))";

const LabelFunctions EvaluatorRelSpeedOvertakeB::labels_ = {
    LabelFunctionPtr(new LeftOfLabelFunction("left_j")),
    LabelFunctionPtr(new FrontOfLabelFunction("in_front_j")),
    LabelFunctionPtr(new BehindOfLabelFunction("behind_j")),
    LabelFunctionPtr(new RelSpeedLabelFunction("speed_diff_j", 10.0 / 3.6))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
