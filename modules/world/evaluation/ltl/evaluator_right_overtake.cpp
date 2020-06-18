// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/ltl/evaluator_right_overtake.hpp"

#include "modules/world/evaluation/ltl/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/dense_traffic_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/front_of_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/right_of_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {

/// Do not overtake to the right, if traffic is not dense!
const char EvaluatorRightOvertake::formula_[] =
    "G (!dense -> !(behind_j#0 & X[!](behind_j#0 U r_v#0 U front_j#0)))";

const LabelFunctions EvaluatorRightOvertake::labels_ = {
    LabelFunctionPtr(new DenseTrafficLabelFunction("dense", 20.0, 8)),
    LabelFunctionPtr(new RightOfLabelFunction("r_v")),
    LabelFunctionPtr(new FrontOfLabelFunction("f_v")),
    LabelFunctionPtr(new BehindOfLabelFunction("b_v"))};

const char EvaluatorRightOvertakeAssumption::formula_[] = "G !dense";

const LabelFunctions EvaluatorRightOvertakeAssumption::labels_ = {
    LabelFunctionPtr(new DenseTrafficLabelFunction("dense", 20.0, 8))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
