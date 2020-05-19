// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_right_overtake.hpp"

#include "modules/world/evaluation/labels/behind_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/front_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/right_of_label_evaluator.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorRightOvertake::formula_[] =
    "G !(b_v#0 & X[!](b_v#0 U r_v#0 U f_v#0))";

const LabelEvaluators EvaluatorRightOvertake::labels_ = {
    LabelEvaluatorPtr(new RightOfLabelEvaluator("r_v")),
    LabelEvaluatorPtr(new FrontOfLabelEvaluator("f_v")),
    LabelEvaluatorPtr(new BehindOfLabelEvaluator("b_v"))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
