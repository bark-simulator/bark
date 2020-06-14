// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_rel_speed_overtake_a.hpp"

#include "modules/world/evaluation/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/rel_speed_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorRelSpeedOvertakeA::formula_[] =
    "G ((left_of#0 & other_near#0) -> rel_speed_gt#0)";

const LabelFunctions EvaluatorRelSpeedOvertakeA::labels_ = {
    LabelFunctionPtr(new LeftOfLabelFunction("left_of")),
    LabelFunctionPtr(new RelSpeedLabelFunction("rel_speed_gt", 20.0 / 3.6)),
    LabelFunctionPtr(new AgentNearLabelFunction("other_near", 6.0))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
