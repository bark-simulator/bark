// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_being_overtaken.hpp"

#include "modules/world/evaluation/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/labels/ego_accelerate_label_function.hpp"
#include "modules/world/evaluation/labels/right_of_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorBeingOvertaken::formula_[] =
    "G ((right_of#0 & other_near#0) -> !accel)";

const LabelFunctions EvaluatorBeingOvertaken::labels_ = {
    LabelFunctionPtr(new RightOfLabelFunction(
        "right_of")),
    LabelFunctionPtr(new EgoAccelerateLabelFunction("accel")),
    LabelFunctionPtr(new AgentNearLabelFunction("other_near"))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
