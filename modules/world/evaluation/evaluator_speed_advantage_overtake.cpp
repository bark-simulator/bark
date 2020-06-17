// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_speed_advantage_overtake.hpp"

#include "modules/world/evaluation/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/labels/front_of_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/rel_speed_label_function.hpp"
#include "modules/world/evaluation/labels/right_of_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorSpeedAdvantageOvertake::formula_[] =
    "G (behind_j#0 & X(behind_j#0 U left_j#0 U in_front_j#0) -> "
    "(speed_diff_j#0 U in_front_j#0))";

const LabelFunctions EvaluatorSpeedAdvantageOvertake::labels_ = {
    LabelFunctionPtr(new LeftOfLabelFunction("left_j")),
    LabelFunctionPtr(new FrontOfLabelFunction("in_front_j")),
    LabelFunctionPtr(new BehindOfLabelFunction("behind_j")),
    LabelFunctionPtr(new RelSpeedLabelFunction("speed_diff_j", 10.0 / 3.6))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
