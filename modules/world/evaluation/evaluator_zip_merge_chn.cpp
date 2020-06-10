// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_zip_merge_chn.hpp"

#include "modules/world/evaluation/labels/agent_at_lane_end_label_function.hpp"
#include "modules/world/evaluation/labels/agent_beyond_point_label_function.hpp"
#include "modules/world/evaluation/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/labels/ego_beyond_point_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/preceding_agent_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {
const char EvaluatorZipMergeChn::formula_[] =
    "(F(i_left_of_k#1 & i_behind_of_k#1 & k_near_i#1 & k_near_lane_end#1) & "
    "j_precedes_i#0 & !merged_i & near_merge_i & (j_precedes_i#0 | merged_j#0) U "
    "merged_i) -> G(merged_i & merged_j#0 -> !j_precedes_i#0)";

const LabelFunctions EvaluatorZipMergeChn::labels_ = {
    LabelFunctionPtr(
        new EgoBeyondPointLabelFunction("near_merge_i", Point2d(-55.0, -164.0))),
    LabelFunctionPtr(
        new AgentBeyondPointLabelFunction("merged_j", Point2d(-22.0, -168.0))),
    LabelFunctionPtr(
        new EgoBeyondPointLabelFunction("merged_i", Point2d(-22.0, -168.0))),
    LabelFunctionPtr(new PrecedingAgentLabelFunction("j_precedes_i")),
    LabelFunctionPtr(new LeftOfLabelFunction("i_left_of_k")),
    LabelFunctionPtr(new BehindOfLabelFunction("i_behind_of_k")),
    LabelFunctionPtr(new AgentNearLabelFunction("k_near_i", 6.0)),
    LabelFunctionPtr(new AgentAtLaneEndLabelFunction("k_near_lane_end", 55.0))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
