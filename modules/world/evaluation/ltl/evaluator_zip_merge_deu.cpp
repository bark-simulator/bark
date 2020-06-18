// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/ltl/evaluator_zip_merge_deu.hpp"

#include "modules/world/evaluation/ltl/labels/agent_at_lane_end_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/agent_beyond_point_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/behind_of_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/ego_beyond_point_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/ltl/labels/preceding_agent_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {

/// Allow other vehicles on ending lanes to merge (zipper merge)
/// This evaluator is parameterized for the DR_DEU_Merging_MT scenario of the
/// INTERACTION dataset
const char EvaluatorZipMergeDeu::formula_[] =
    "G (((left#1 & behind#1 & near#1 & near_lane_end#1) & "
    "precedes#0 & !ego_merged & (precedes#0 | merged#0) U "
    "ego_merged) -> G(ego_merged & merged#0 -> !precedes#0))";

const LabelFunctions EvaluatorZipMergeDeu::labels_ = {
    LabelFunctionPtr(
        new AgentBeyondPointLabelFunction("merged", Point2d(966, 1008))),
    LabelFunctionPtr(
        new EgoBeyondPointLabelFunction("ego_merged", Point2d(966, 1008))),
    LabelFunctionPtr(new PrecedingAgentLabelFunction("precedes")),
    LabelFunctionPtr(new LeftOfLabelFunction("left")),
    LabelFunctionPtr(new BehindOfLabelFunction("behind")),
    LabelFunctionPtr(new AgentNearLabelFunction("near", 6.0)),
    LabelFunctionPtr(new AgentAtLaneEndLabelFunction("near_lane_end", 55.0))};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
