// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/evaluator_right_overtake.hpp"

#include "bark/world/evaluation/ltl/labels/behind_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/dense_traffic_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/front_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/right_of_label_function.hpp"

namespace bark {
namespace world {
namespace evaluation {

/// Do not overtake to the right, if traffic is not dense!
const char EvaluatorRightOvertake::formula_[] =
  "G (!dense -> !(behind#0 & X[!](behind#0 U right#0 U front#0)))";

const LabelFunctions EvaluatorRightOvertake::labels_ = {
    LabelFunctionPtr(new DenseTrafficLabelFunction("dense", 20.0, 8)),
  LabelFunctionPtr(new RightOfLabelFunction("right")),
  LabelFunctionPtr(new FrontOfLabelFunction("front")),
  LabelFunctionPtr(new BehindOfLabelFunction("behind"))};

const char EvaluatorRightOvertakeAssumption::formula_[] = "G !dense";

const LabelFunctions EvaluatorRightOvertakeAssumption::labels_ = {
    LabelFunctionPtr(new DenseTrafficLabelFunction("dense", 20.0, 8))};

}  // namespace evaluation
}  // namespace world
}  // namespace bark
