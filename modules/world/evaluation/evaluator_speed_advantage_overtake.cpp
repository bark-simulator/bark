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

/// Overtake with a minimum speed advantage of 10 km/h!
const char EvaluatorSpeedAdvantageOvertake::formula_[] =
    "G (behind#0 & X(behind#0 U left#0 U front#0) -> "
    "(speed_diff#0 U front#0))";

const LabelFunctions EvaluatorSpeedAdvantageOvertake::labels_ = {
    LabelFunctionPtr(new LeftOfLabelFunction("left")),
    LabelFunctionPtr(new FrontOfLabelFunction("front")),
    LabelFunctionPtr(new BehindOfLabelFunction("behind")),
    LabelFunctionPtr(new RelSpeedLabelFunction("speed_diff", 10.0 / 3.6))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
