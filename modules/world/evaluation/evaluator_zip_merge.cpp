// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_zip_merge.hpp"

#include "modules/world/evaluation/labels/agent_beyond_point_label_function.hpp"
#include "modules/world/evaluation/labels/direct_front_of_label_function.hpp"
#include "modules/world/evaluation/labels/ego_beyond_point_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorZipMerge::formula_[] =
    "(in_direct_front_x#0 & !merged_e & (in_direct_front_x#0 | merged_x#0) U "
    "merged_e) -> G(merged_e & merged_x#0 -> !in_direct_front_x#0)";

const LabelFunctions EvaluatorZipMerge::labels_ = {
    LabelFunctionPtr(
        new AgentBeyondPointLabelFunction("merged_x", Point2d(968, 1008))),
    LabelFunctionPtr(
        new EgoBeyondPointLabelFunction("merged_e", Point2d(968, 1008))),
    LabelFunctionPtr(new DirectFrontOfLabelFunction("in_direct_front_x"))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
