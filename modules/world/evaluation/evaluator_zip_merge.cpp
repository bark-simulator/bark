// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_zip_merge.hpp"

#include "modules/world/evaluation/labels/agent_at_lane_end_label_function.hpp"
#include "modules/world/evaluation/labels/agent_beyond_point_label_function.hpp"
#include "modules/world/evaluation/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/labels/direct_front_of_label_function.hpp"
#include "modules/world/evaluation/labels/ego_beyond_point_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorZipMerge::formula_[] =
    "(i_left_of_k#1 & i_behind_of_k#1 & k_near_i#1 & k_near_lane_end#1 & "
    "j_in_direct_front#0 & !merged_i & (j_in_direct_front#0 | merged_j#0) U "
    "merged_i) -> G(merged_i & merged_j#0 -> !j_in_direct_front#0)";
// const char EvaluatorZipMerge::formula_[] =
//    "G !(i_left_of_k#1 & i_behind_of_k#1 & k_near_i#1 & k_near_lane_end#1 &
//    j_in_direct_front#0 & !merged_i & (j_in_direct_front#0 | merged_j#0) U "
//    "merged_i)";

const LabelFunctions EvaluatorZipMerge::labels_ = {
    LabelFunctionPtr(
        new AgentBeyondPointLabelFunction("merged_j", Point2d(968, 1008))),
    LabelFunctionPtr(
        new EgoBeyondPointLabelFunction("merged_i", Point2d(968, 1008))),
    LabelFunctionPtr(new DirectFrontOfLabelFunction("j_in_direct_front")),
    LabelFunctionPtr(new LeftOfLabelFunction("i_left_of_k")),
    LabelFunctionPtr(new BehindOfLabelFunction("i_behind_of_k")),
    LabelFunctionPtr(new AgentNearLabelFunction("k_near_i", 6.0)),
    LabelFunctionPtr(new AgentAtLaneEndLabelFunction("k_near_lane_end", 40.0))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
